"""RTCM output transports (sinks).

Endpoints are configured as URIs:

    udp://HOST:PORT              datagram per frame; broadcast addresses allowed
    tcpserver://BIND:PORT        rovers connect to the base
    tcp://HOST:PORT              base connects out to a relay / rover
    serial:///dev/ttyX?baud=N    radio modem

Per-endpoint options go in the query string:
    reconnect_s=2.0       wait between (re)connection attempts (tcp)
    send_timeout_s=0.05   longest a sink may block on one client (tcp, tcpserver, serial)
    baud=57600            serial endpoints
e.g.  tcp://192.168.1.10:2101?send_timeout_s=0.1

A sink's `send()` blocks for at most send_timeout_s per client; a TCP client
that does not accept the data within that time is disconnected.
"""
import socket
import threading
from dataclasses import dataclass, field
from typing import Dict, List
from urllib.parse import parse_qs, urlparse

import serial

DEFAULT_RECONNECT_S = 2.0
DEFAULT_SEND_TIMEOUT_S = 0.05
DEFAULT_SERIAL_BAUD = 57600

SCHEMES = ('udp', 'tcp', 'tcpserver', 'serial')


@dataclass
class Endpoint:
    scheme: str
    host: str = ''
    port: int = 0
    path: str = ''
    options: Dict[str, str] = field(default_factory=dict)
    uri: str = ''


def parse_endpoint(uri: str) -> Endpoint:
    u = urlparse(uri.strip())
    scheme = u.scheme.lower()
    if scheme not in SCHEMES:
        raise ValueError(f'unsupported RTCM endpoint scheme in {uri!r}')
    options = {k: v[-1] for k, v in parse_qs(u.query).items()}
    if scheme == 'serial':
        if not u.path:
            raise ValueError(f'serial endpoint needs a device path: {uri!r}')
        return Endpoint(scheme, path=u.path, options=options, uri=uri)
    if not u.port:
        raise ValueError(f'endpoint needs a port: {uri!r}')
    host = u.hostname or ('0.0.0.0' if scheme in ('udp', 'tcpserver') else '')
    if not host:
        raise ValueError(f'endpoint needs a host: {uri!r}')
    return Endpoint(scheme, host=host, port=u.port, options=options, uri=uri)


class Transport:
    """Common bookkeeping for sinks."""

    def __init__(self, endpoint: Endpoint, logger):
        self.endpoint = endpoint
        self.log = logger
        opts = endpoint.options
        try:
            self.reconnect_s = float(opts.get('reconnect_s', DEFAULT_RECONNECT_S))
            self.send_timeout_s = float(opts.get('send_timeout_s', DEFAULT_SEND_TIMEOUT_S))
            self.baudrate = int(opts.get('baud', DEFAULT_SERIAL_BAUD))
        except ValueError as e:
            raise ValueError(f'bad option in {endpoint.uri!r}: {e}')
        self.bytes = 0
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []

    @property
    def name(self) -> str:
        return self.endpoint.uri

    def _spawn(self, target):
        t = threading.Thread(target=target, name=f'rtcm:{self.name}', daemon=True)
        self._threads.append(t)
        t.start()

    def start(self):
        pass

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)

    def status(self) -> str:
        return 'ok'


# --------------------------------------------------------------------- sinks
class Sink(Transport):

    def send(self, data: bytes):
        raise NotImplementedError


class UdpSink(Sink):

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._addr = (self.endpoint.host, self.endpoint.port)

    def send(self, data):
        try:
            self._sock.sendto(data, self._addr)
            self.bytes += len(data)
        except OSError as e:
            self.log.warn(f'RTCM {self.name}: {e}', throttle_duration_sec=5.0)

    def stop(self):
        super().stop()
        self._sock.close()


class TcpServerSink(Sink):

    def start(self):
        self._clients: Dict[socket.socket, tuple] = {}
        self._lock = threading.Lock()
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((self.endpoint.host, self.endpoint.port))
        self._server.listen(8)
        self._server.settimeout(0.5)
        self._spawn(self._accept_loop)

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                conn, addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.settimeout(self.send_timeout_s)
            with self._lock:
                self._clients[conn] = addr
            self.log.info(f'RTCM {self.name}: client {addr[0]}:{addr[1]} connected')

    def send(self, data):
        with self._lock:
            clients = list(self._clients.items())
        for conn, addr in clients:
            try:
                conn.sendall(data)
                self.bytes += len(data)
            except OSError:
                self.log.warn(f'RTCM {self.name}: dropping client {addr[0]}:{addr[1]}')
                with self._lock:
                    self._clients.pop(conn, None)
                conn.close()

    def status(self):
        return f'{len(self._clients)} client(s)'

    def stop(self):
        super().stop()
        self._server.close()
        with self._lock:
            for conn in self._clients:
                conn.close()
            self._clients.clear()


class TcpClientSink(Sink):

    def start(self):
        self._sock = None
        self._lock = threading.Lock()
        self._spawn(self._connect_loop)

    def _connect_loop(self):
        while not self._stop.is_set():
            if self._sock is None:
                try:
                    s = socket.create_connection((self.endpoint.host, self.endpoint.port), timeout=2.0)
                    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    s.settimeout(self.send_timeout_s)
                    with self._lock:
                        self._sock = s
                    self.log.info(f'RTCM {self.name}: connected')
                except OSError:
                    pass
            self._stop.wait(self.reconnect_s)

    def send(self, data):
        with self._lock:
            s = self._sock
        if s is None:
            return
        try:
            s.sendall(data)
            self.bytes += len(data)
        except OSError as e:
            self.log.warn(f'RTCM {self.name}: {e}, reconnecting')
            with self._lock:
                self._sock = None
            s.close()

    def status(self):
        return 'connected' if self._sock is not None else 'connecting'

    def stop(self):
        super().stop()
        if self._sock is not None:
            self._sock.close()


class SerialSink(Sink):

    def start(self):
        self._port = serial.Serial(self.endpoint.path, self.baudrate,
                                   write_timeout=self.send_timeout_s)

    def send(self, data):
        try:
            self._port.write(data)
            self.bytes += len(data)
        except (serial.SerialException, OSError) as e:
            self.log.warn(f'RTCM {self.name}: {e}', throttle_duration_sec=5.0)

    def stop(self):
        super().stop()
        self._port.close()


# ------------------------------------------------------------------- sources


# ------------------------------------------------------------------ factory
_SINKS = {'udp': UdpSink, 'tcpserver': TcpServerSink, 'tcp': TcpClientSink, 'serial': SerialSink}


def create_sink(uri: str, logger) -> Sink:
    ep = parse_endpoint(uri)
    return _SINKS[ep.scheme](ep, logger)
