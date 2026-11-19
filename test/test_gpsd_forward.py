"""Tests that the node forwards the receiver's NMEA to gpsd over UDP (use_gpsd_udp)."""
import os
import pty
import signal
import socket
import subprocess
import sys
import time
import tty

from rtk_base.protocol.checksums import nmea_checksum


def nmea(body):
    return f'${body}*{nmea_checksum(body):02X}\r\n'.encode()


GGA = nmea('GNGGA,173144.00,4129.4758,N,07125.3116,W,1,12,0.6,16.28,M,-30.39,M,,')
GSV = nmea('GPGSV,1,1,01,01,34,253,42,1')


def test_nmea_lines_matching_the_patterns_are_forwarded():
    master, slave = pty.openpty()
    tty.setraw(slave)
    gpsd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gpsd.bind(('127.0.0.1', 0))
    gpsd.settimeout(0.2)
    node = subprocess.Popen(
        [sys.executable, '-m', 'rtk_base.base_node', '--ros-args',
         '-r', '__ns:=/gpsd_test', '-p', f'device:={os.ttyname(slave)}',
         '-p', 'receiver:=generic', '-p', 'use_gpsd_udp:=true',
         '-p', f'virtual_port:={gpsd.getsockname()[1]}'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    received = []
    try:
        end = time.monotonic() + 20
        while time.monotonic() < end and not any(r.startswith(b'$GNGGA') for r in received):
            os.write(master, GGA + GSV)
            try:
                while True:
                    received.append(gpsd.recv(4096))
            except socket.timeout:
                pass
        node.send_signal(signal.SIGINT)
        output = node.communicate(timeout=20)[0]
    finally:
        node.kill()
        gpsd.close()
        os.close(master)
        os.close(slave)
    assert GGA in received, output
    assert not any(b'GPGSV' in r for r in received)  # default gpsd_udp_patterns: ['$GN']
