"""Forward RTCM from a base receiver's serial port to UDP, TCP or serial outputs,
without ROS.

    python3 -m rtk_base.tools.relay --device /dev/ttyACM0 --baud 115200 \\
        --output tcpserver://0.0.0.0:2101 --output udp://<broadcast_address>:2101

    python3 -m rtk_base.tools.relay @/home/user/rtk/relay.conf

Uses the same serial link, framer and RTCM outputs as the ROS node.  Complete
RTCM frames with a valid CRC are forwarded; NMEA and other text is discarded.
The serial port is reopened after a disconnect.

By default the receiver is not configured (set up base mode and RTCM output
with the receiver's own tools).  With `--receiver unicore` the relay sends the
UM98x base commands after every (re)connect: survey-in (`--survey-s`) or a fixed
position (`--fixed-position LAT LON ALT`), and one RTCM<n> command per
`--rtcm-message`.

Arguments can be read from a file with @FILE, one option per line.  Lines
starting with # and blank lines are skipped.  `--name=value` keeps the whole
value (spaces included); `--name v1 v2 v3` gives several values, e.g.:

    --device=/dev/ttyACM0
    --baud=115200
    --output=tcpserver://0.0.0.0:2101
    # --fixed-position LAT LON HEIGHT

Runs with Python 3 and pyserial (Debian/Ubuntu: python3-serial); ROS is
optional.
"""
import argparse
import logging
import signal
import sys
import threading
import time

from rtk_base.protocol.framer import FrameType, StreamFramer
from rtk_base.receivers import (BaseStationSettings, build_commands,
                                create_receiver, parse_rtcm_message_list)
from rtk_base.receivers.base import DEFAULT_RTCM_MESSAGES
from rtk_base.rtcm.router import RtcmRouter
from rtk_base.rtcm.transport import create_sink
from rtk_base.serial_link import SerialLink


class Logger:
    """Python logging with the interface the shared modules expect
    (warn() and throttle_duration_sec, as in rclpy)."""

    def __init__(self, name='rtk_base_relay'):
        self._log = logging.getLogger(name)
        self._last = {}
        self._lock = threading.Lock()

    def _emit(self, level, msg, throttle_duration_sec=0.0):
        if throttle_duration_sec:
            now = time.monotonic()
            with self._lock:
                if now - self._last.get((level, msg), -1e9) < throttle_duration_sec:
                    return
                self._last[(level, msg)] = now
        self._log.log(level, msg)

    def debug(self, msg, **kw):
        self._emit(logging.DEBUG, msg, **kw)

    def info(self, msg, **kw):
        self._emit(logging.INFO, msg, **kw)

    def warn(self, msg, **kw):
        self._emit(logging.WARNING, msg, **kw)

    warning = warn

    def error(self, msg, **kw):
        self._emit(logging.ERROR, msg, **kw)


class Relay:

    def __init__(self, device, baudrate, outputs, logger, serial_reconnect_s=1.0,
                 no_rtcm_warning_s=10.0, receiver=None, base=None, save_config=False):
        self.log = logger
        self.no_rtcm_warning_s = no_rtcm_warning_s
        self.receiver = receiver
        self.base = base or BaseStationSettings()
        self.save_config = save_config
        self.framer = StreamFramer()
        self.router = RtcmRouter(logger)
        for uri in outputs:
            self.router.add_sink(create_sink(uri, logger))
        self.link = SerialLink(device, baudrate, logger, on_data=self._on_serial_data,
                               on_connect=self._on_connect, reconnect_s=serial_reconnect_s)
        self._started = time.monotonic()

    def _on_connect(self):
        """Runs on the serial thread after each (re)connect."""
        if self.receiver is None:
            return
        cmds = build_commands(self.receiver, self.base, [], save=self.save_config)
        if cmds:
            self.log.info('configuring base station: ' +
                          ' '.join(self.receiver.describe_base(self.base)))
        for cmd in cmds:
            self.log.info(f'sending: {cmd}')
            if not self.link.write_line(cmd):
                return

    def _on_serial_data(self, data):
        for frame in self.framer.feed(data):
            if frame.type is FrameType.RTCM:
                self.router.publish(frame.data)
            elif (self.receiver is not None and frame.type is FrameType.NMEA
                  and self.receiver.is_command_response(frame.text)):
                self.log.info(f'receiver replied: {frame.text}')

    def start(self):
        self.router.start()
        self.link.start()

    def stop(self):
        self.link.stop()
        self.router.stop()

    def status(self):
        """One line summary; also updates the output rate."""
        out = self.router.outgoing
        out.roll()
        parts = [
            f'serial {"connected" if self.link.is_open else "disconnected"}',
            f'rtcm {out.rate_bps:.0f} B/s',
            'msgs ' + (' '.join(f'{k}:{v}' for k, v in sorted(out.message_counts.items())) or '-'),
        ]
        parts += [f'{s.name} {s.status()}' for s in self.router.sinks]
        if self.router.base_station:
            sid, lat, lon, h = self.router.base_station
            parts.append(f'base {sid} {lat:.8f} {lon:.8f} {h:.3f}m')
        return ' | '.join(parts)

    def check_rtcm(self):
        """Warn when the receiver has not produced RTCM for a while."""
        age = self.router.outgoing.age()
        silent = age if age is not None else time.monotonic() - self._started
        if self.link.is_open and silent > self.no_rtcm_warning_s:
            self.log.warn(f'no RTCM from the receiver for {silent:.0f} s: is base mode and '
                          'RTCM output enabled on this port?', throttle_duration_sec=30.0)


class _ArgumentFileParser(argparse.ArgumentParser):
    """Read @FILE lines: skip blank lines and # comments, keep `--name=value` whole.

    Any other line is split on whitespace, for example `--fixed-position LAT LON H`.
    """

    def convert_arg_line_to_args(self, arg_line):
        line = arg_line.strip()
        if not line or line.startswith('#'):
            return []
        if line.startswith('-') and '=' in line.split()[0]:
            return [line]
        return line.split()


def parse_args(argv=None):
    parser = _ArgumentFileParser(
        prog='rtk_base_relay', fromfile_prefix_chars='@',
        description='Forward RTCM from a base receiver to UDP / TCP / serial outputs, without ROS.')
    parser.add_argument('--device', required=True, help='receiver serial device')
    parser.add_argument('--baud', type=int, default=115200, help='receiver baud rate')
    parser.add_argument('--output', action='append', required=True,
                        help='RTCM output URI (repeatable): udp://HOST:PORT, '
                             'tcpserver://BIND:PORT, tcp://HOST:PORT, serial:///dev/ttyX?baud=N')
    parser.add_argument('--receiver', choices=['generic', 'unicore'], default='generic',
                        help='unicore: send the UM98x base commands after every connect; '
                             'generic: do not configure the receiver (default)')
    parser.add_argument('--receiver-port', default='',
                        help="receiver-side port name used in RTCM/log commands, e.g. COM1 "
                             "(default: the receiver's current port)")
    parser.add_argument('--survey-s', type=int, default=120,
                        help='survey-in time in seconds (unicore, default 120)')
    parser.add_argument('--fixed-position', type=float, nargs=3,
                        metavar=('LAT', 'LON', 'ALT'),
                        help='use a surveyed position instead of survey-in: degrees, degrees, '
                             'metres (unicore)')
    parser.add_argument('--mode-command', default='',
                        help='send this command verbatim instead of the generated MODE BASE one')
    parser.add_argument('--rtcm-message', action='append', metavar='NUMBER:PERIOD_S',
                        help='RTCM message to enable (repeatable, unicore); '
                             'default 1006:10 1033:10 1074:1 1084:1 1094:1 1124:1')
    parser.add_argument('--save-config', action='store_true',
                        help='also save the receiver configuration (writes receiver flash)')
    parser.add_argument('--status-period', type=float, default=10.0,
                        help='seconds between status lines, 0 disables them')
    parser.add_argument('--serial-reconnect-s', type=float, default=1.0)
    parser.add_argument('--no-rtcm-warning-s', type=float, default=10.0,
                        help='warn when no RTCM arrives for this long')
    parser.add_argument('--log-level', default='INFO',
                        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'])
    return parser.parse_args(argv)


def main(argv=None, stop_event=None):
    args = parse_args(argv)
    logging.basicConfig(level=args.log_level, stream=sys.stdout,
                        format='%(asctime)s %(levelname)s %(message)s')
    log = Logger()
    try:
        receiver = None if args.receiver == 'generic' else create_receiver(
            args.receiver, args.receiver_port)
        fixed = args.fixed_position
        base = BaseStationSettings(
            mode_command=args.mode_command.strip(),
            survey_in_min_duration_s=args.survey_s,
            use_fixed_position=fixed is not None,
            fixed_latitude_deg=fixed[0] if fixed else 0.0,
            fixed_longitude_deg=fixed[1] if fixed else 0.0,
            fixed_altitude_m=fixed[2] if fixed else 0.0,
            rtcm_messages=(parse_rtcm_message_list(args.rtcm_message)
                           if args.rtcm_message else list(DEFAULT_RTCM_MESSAGES)))
        relay = Relay(args.device, args.baud, args.output, log,
                      serial_reconnect_s=args.serial_reconnect_s,
                      no_rtcm_warning_s=args.no_rtcm_warning_s,
                      receiver=receiver, base=base, save_config=args.save_config)
        relay.start()
    except (ValueError, OSError) as e:
        log.error(f'cannot start: {e}')
        return 2

    stop = stop_event or threading.Event()
    if stop_event is None:
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stop.set())
    log.info(f'relaying RTCM from {args.device} at {args.baud} baud to ' + ', '.join(args.output))

    next_status = time.monotonic() + args.status_period
    while not stop.wait(1.0):
        relay.check_rtcm()
        if args.status_period > 0 and time.monotonic() >= next_status:
            log.info(relay.status())
            next_status += args.status_period
    relay.stop()
    log.info('stopped')
    return 0


if __name__ == '__main__':
    sys.exit(main())
