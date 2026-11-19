"""Tests for the RTCM 3 helpers, the router and the output transports."""
import socket
import time

import pytest

from rtcm_helpers import lla_to_ecef, message, station_1005
from rtk_base.rtcm import rtcm3
from rtk_base.rtcm.router import RtcmRouter
from rtk_base.rtcm.transport import create_sink, parse_endpoint


class Log:
    def __init__(self):
        self.lines = []

    def _log(self, msg, **_):
        self.lines.append(msg)

    debug = info = warn = warning = error = _log


def free_port(kind=socket.SOCK_STREAM):
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def wait_for(cond, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


# ------------------------------------------------------------------- rtcm3
def test_station_position_roundtrip():
    lat, lon, h = 41.49126464, -71.42186016, -14.11
    x, y, z = lla_to_ecef(lat, lon, h)
    fr = station_1005(42, x, y, z)
    assert rtcm3.message_type(fr) == 1005
    sid, dx, dy, dz = rtcm3.station_arp_ecef(fr)
    assert sid == 42
    assert (dx, dy, dz) == pytest.approx((x, y, z), abs=1e-4)
    # RTCM resolution is 0.1 mm per axis
    lat2, lon2, h2 = rtcm3.ecef_to_lla(dx, dy, dz)
    assert (lat2, lon2) == pytest.approx((lat, lon), abs=1e-8)
    assert h2 == pytest.approx(h, abs=2e-4)
    assert rtcm3.station_arp_ecef(message(1074)) is None


# ------------------------------------------------------------------ router
def test_router_reports_base_station():
    log = Log()
    r = RtcmRouter(log)
    x, y, z = lla_to_ecef(41.5, -71.4, 10.0)
    r.publish(station_1005(7, x, y, z))
    assert r.base_station[0] == 7
    assert r.base_station[1:3] == pytest.approx((41.5, -71.4), abs=1e-8)
    assert r.base_station[3] == pytest.approx(10.0, abs=2e-4)
    assert any('base station 7' in line for line in log.lines)


def test_router_counts_messages_and_feeds_every_sink():
    class Collect:
        name = 'collect'

        def __init__(self):
            self.frames = []

        def send(self, data):
            self.frames.append(data)

    a, b = Collect(), Collect()
    r = RtcmRouter(Log())
    r.add_sink(a)
    r.add_sink(b)
    frames = [message(1074, 30), message(1084, 30), message(1074, 30)]
    for f in frames:
        r.publish(f)
    assert a.frames == b.frames == frames
    assert r.outgoing.message_counts[1074] == 2 and r.outgoing.message_counts[1084] == 1
    assert r.outgoing.age() is not None


# -------------------------------------------------------------- endpoints
def test_endpoint_parsing():
    ep = parse_endpoint('serial:///dev/ttyUSB1?baud=57600')
    assert (ep.path, ep.options['baud']) == ('/dev/ttyUSB1', '57600')
    assert parse_endpoint('udp://:2101').host == '0.0.0.0'
    assert parse_endpoint('tcpserver://:2101').host == '0.0.0.0'
    ep = parse_endpoint('tcp://192.168.1.10:2101?send_timeout_s=0.1')
    assert (ep.host, ep.port, ep.options['send_timeout_s']) == ('192.168.1.10', 2101, '0.1')
    # ntrip is a rover input, not an output
    for bad in ('http://x:1', 'ntrip://h:1/M', 'tcp://host', 'tcp://:2101', 'serial://'):
        with pytest.raises(ValueError):
            parse_endpoint(bad)


def test_endpoint_options():
    log = Log()
    sink = create_sink('tcpserver://127.0.0.1:1?send_timeout_s=0.01', log)
    assert sink.send_timeout_s == 0.01
    assert create_sink('tcp://127.0.0.1:1?reconnect_s=0.25', log).reconnect_s == 0.25
    with pytest.raises(ValueError):
        create_sink('tcp://127.0.0.1:1?reconnect_s=soon', log)


# ------------------------------------------------------------------- sinks
FRAMES = [message(1074 + 10 * i, 50) for i in range(4)]


def test_udp_sink():
    port = free_port(socket.SOCK_DGRAM)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(('127.0.0.1', port))
    rx.settimeout(2.0)
    sink = create_sink(f'udp://127.0.0.1:{port}', Log())
    sink.start()
    try:
        for f in FRAMES:
            sink.send(f)
        assert [rx.recv(4096) for _ in FRAMES] == FRAMES
    finally:
        sink.stop()
        rx.close()


def test_tcpserver_sink_serves_clients():
    port = free_port()
    sink = create_sink(f'tcpserver://127.0.0.1:{port}', Log())
    sink.start()
    try:
        client = socket.create_connection(('127.0.0.1', port))
        assert wait_for(lambda: sink.status() == '1 client(s)')
        for f in FRAMES:
            sink.send(f)
        want, got = b''.join(FRAMES), b''
        client.settimeout(2.0)
        while len(got) < len(want):
            got += client.recv(4096)
        assert got == want
        client.close()
    finally:
        sink.stop()


def test_tcp_client_sink_connects_out_and_reconnects():
    port = free_port()
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', port))
    server.listen(1)
    server.settimeout(3.0)
    sink = create_sink(f'tcp://127.0.0.1:{port}?reconnect_s=0.05', Log())
    sink.start()
    try:
        conn, _ = server.accept()
        assert wait_for(lambda: sink.status() == 'connected')
        sink.send(FRAMES[0])
        conn.settimeout(2.0)
        assert conn.recv(4096) == FRAMES[0]
        conn.close()
        for _ in range(50):              # first sends to the closed peer fail
            sink.send(FRAMES[1])
        conn2, _ = server.accept()
        assert wait_for(lambda: sink.status() == 'connected')
        sink.send(FRAMES[2])
        conn2.settimeout(2.0)
        assert FRAMES[2] in conn2.recv(4096)
        conn2.close()
    finally:
        sink.stop()
        server.close()


def test_tcpserver_sink_drops_dead_client_and_keeps_serving():
    port = free_port()
    log = Log()
    sink = create_sink(f'tcpserver://127.0.0.1:{port}', log)
    sink.start()
    try:
        dead = socket.create_connection(('127.0.0.1', port))
        live = socket.create_connection(('127.0.0.1', port))
        assert wait_for(lambda: sink.status() == '2 client(s)')
        dead.close()
        for _ in range(50):          # first sends to the closed peer fail
            sink.send(message(1074, 100))
        assert wait_for(lambda: sink.status() == '1 client(s)')
        live.settimeout(1.0)
        assert live.recv(10)
        live.close()
    finally:
        sink.stop()
