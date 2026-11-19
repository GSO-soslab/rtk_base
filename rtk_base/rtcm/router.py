"""Send the RTCM frames the base receiver produces to the output transports,
and keep statistics.

    receiver serial --frames--> RtcmRouter.publish() --> every sink

`publish()` runs on the thread that read the data (the serial reader) and
writes directly to the sinks.
"""
import threading
import time
from collections import Counter
from typing import List, Optional, Tuple

from rtk_base.rtcm import rtcm3
from rtk_base.rtcm.transport import Sink


class RtcmStats:

    def __init__(self):
        self._lock = threading.Lock()
        self.message_counts = Counter()
        self.last_time = 0.0
        self._window_start = time.monotonic()
        self._window_bytes = 0
        self.rate_bps = 0.0

    def add_frame(self, frame: bytes):
        with self._lock:
            self.message_counts[rtcm3.message_type(frame)] += 1
            self._window_bytes += len(frame)
            self.last_time = time.monotonic()

    def roll(self):
        """Update `rate_bps`; call periodically (e.g. from the diagnostics timer)."""
        with self._lock:
            now = time.monotonic()
            dt = now - self._window_start
            if dt > 0:
                self.rate_bps = self._window_bytes / dt
            self._window_start, self._window_bytes = now, 0

    def age(self) -> Optional[float]:
        return time.monotonic() - self.last_time if self.last_time else None


class RtcmRouter:

    def __init__(self, logger):
        self.log = logger
        self.sinks: List[Sink] = []
        self.outgoing = RtcmStats()
        self.base_station: Optional[Tuple[int, float, float, float]] = None  # id, lat, lon, h

    def add_sink(self, sink: Sink):
        self.sinks.append(sink)

    def start(self):
        for sink in self.sinks:
            sink.start()

    def stop(self):
        for sink in self.sinks:
            try:
                sink.stop()
            except Exception as e:  # keep shutting the others down
                self.log.warn(f'RTCM {sink.name}: error on stop: {e}')

    def publish(self, frame: bytes):
        """A complete RTCM frame produced by the local receiver."""
        self.outgoing.add_frame(frame)
        self._note_station(frame)
        for sink in self.sinks:
            sink.send(frame)

    def _note_station(self, frame: bytes):
        arp = rtcm3.station_arp_ecef(frame)
        if arp is None:
            return
        station_id, x, y, z = arp
        lat, lon, h = rtcm3.ecef_to_lla(x, y, z)
        first = self.base_station is None
        self.base_station = (station_id, lat, lon, h)
        if first:
            self.log.info(f'RTCM base station {station_id} position: '
                          f'{lat:.8f}, {lon:.8f}, {h:.3f} m (ellipsoid)')
