"""Convert the base receiver's NMEA sentences into a NavSatFix.

Same GGA + GST handling and covariance modes as the rover driver
(unicore_rtk_driver), so base and rover recordings are comparable.  The node
calls `handle_nmea()` with each line and its arrival stamp; the processor
validates the line, parses it and passes the resulting message to the
`publish(name, msg)` callback.  Output name: `fix`.
"""
import math
from dataclasses import dataclass

from builtin_interfaces.msg import Time
from sensor_msgs.msg import NavSatFix, NavSatStatus

from rtk_base.protocol import nmea_parser
from rtk_base.protocol.checksums import nmea_is_valid

FIX_COVARIANCE_MODES = ('hdop_scaled', 'gst')

# GGA quality indicator -> name used in diagnostics
GGA_SOLUTION_NAMES = {
    0: 'no fix', 1: 'single', 2: 'DGPS', 3: 'PPS', 4: 'RTK fixed', 5: 'RTK float',
    6: 'dead reckoning', 7: 'manual position', 8: 'simulation', 9: 'SBAS',
}


@dataclass
class Config:
    gps_frame_id: str = 'rtk_base'
    use_gnss_time: bool = False
    epe_quality0: float = 1000000.0
    epe_quality1: float = 4.0
    epe_quality2: float = 0.1
    epe_quality4: float = 0.02
    epe_quality5: float = 4.0
    epe_quality9: float = 3.0
    # 'hdop_scaled': var = (hdop * gst_std)^2, alt uses 2*hdop.
    # 'gst': var = gst_std^2 (GST std devs are already 1-sigma metres).
    fix_covariance_mode: str = 'hdop_scaled'
    # True: only publish a fix when a valid GGA arrived since the last published fix.
    strict_validation: bool = False


def _time_to_float(t: Time) -> float:
    return t.sec + t.nanosec * 1e-9


class FixProcessor:

    def __init__(self, cfg: Config, publish, logger):
        """
        Args:
            cfg: Config
            publish: callable(name: str, msg) -> None
            logger: rclpy-style logger (debug/info/warn/error)
        """
        if cfg.fix_covariance_mode not in FIX_COVARIANCE_MODES:
            raise ValueError(f'fix_covariance_mode must be one of {FIX_COVARIANCE_MODES}')
        self.cfg = cfg
        self.publish = publish
        self.log = logger

        self.hdop = float('nan')
        self.gga_quality = None
        self.num_satellites = 0
        self.valid_fix = False
        self._gga_fresh = False

        # fix type from GGA -> (default EPE, NavSatStatus, covariance type)
        self.gps_qualities = {
            -1: [cfg.epe_quality0, NavSatStatus.STATUS_NO_FIX, NavSatFix.COVARIANCE_TYPE_UNKNOWN],
            0: [cfg.epe_quality0, NavSatStatus.STATUS_NO_FIX, NavSatFix.COVARIANCE_TYPE_UNKNOWN],
            1: [cfg.epe_quality1, NavSatStatus.STATUS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED],
            2: [cfg.epe_quality2, NavSatStatus.STATUS_SBAS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED],
            4: [cfg.epe_quality4, NavSatStatus.STATUS_GBAS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED],
            5: [cfg.epe_quality5, NavSatStatus.STATUS_GBAS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED],
            9: [cfg.epe_quality9, NavSatStatus.STATUS_GBAS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED],
        }

        self.fix_msg = NavSatFix()

    def handle_nmea(self, line: str, stamp: Time) -> bool:
        if not nmea_is_valid(line):
            self.log.warn(f'Received a sentence with an invalid checksum: {line!r}')
            return False

        parsed = nmea_parser.parse_nmea_sentence(line, now=_time_to_float(stamp))
        if not parsed:
            self.log.debug(f'Failed to parse NMEA sentence: {line}')
            return False

        if 'GGA' in parsed:
            return self._gga(parsed['GGA'], stamp)
        if 'GST' in parsed:
            return self._gst(parsed['GST'])
        return True

    def _gga(self, data, stamp):
        if math.isnan(data['utc_time'][0]):
            self.log.warn('Time in the NMEA GGA sentence is NOT valid')
            return False

        fix = self.fix_msg
        if self.cfg.use_gnss_time:
            fix.header.stamp = Time(sec=data['utc_time'][0], nanosec=data['utc_time'][1])
        else:
            fix.header.stamp = stamp
        fix.header.frame_id = self.cfg.gps_frame_id

        fix_type = data['fix_type']
        self.gga_quality = fix_type
        self.num_satellites = data['num_satellites']
        if fix_type not in self.gps_qualities:
            fix_type = -1
        gps_qual = self.gps_qualities[fix_type]
        self.valid_fix = fix_type > 0

        latitude = data['latitude']
        if data['latitude_direction'] == 'S':
            latitude = -latitude
        longitude = data['longitude']
        if data['longitude_direction'] == 'W':
            longitude = -longitude
        # GGA field 9 is height above mean sea level and field 11 the geoid
        # separation, so their sum is the height above the WGS84 ellipsoid,
        # which is what NavSatFix.altitude is defined as.
        altitude = data['altitude'] + data['mean_sea_level']
        self.hdop = data['hdop']

        fix.status.status = gps_qual[1]
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance_type = gps_qual[2]
        fix.latitude = latitude
        fix.longitude = longitude
        fix.altitude = altitude

        self._gga_fresh = True
        return True

    def _gst(self, data):
        fix = self.fix_msg
        if self.cfg.strict_validation and not (self._gga_fresh and not math.isnan(self.hdop)):
            self.log.debug('GST without a fresh valid GGA, fix not published')
            return False

        lon_std, lat_std, alt_std = data['lon_std_dev'], data['lat_std_dev'], data['alt_std_dev']
        if self.cfg.fix_covariance_mode == 'gst':
            fix.position_covariance[0] = lon_std ** 2
            fix.position_covariance[4] = lat_std ** 2
            fix.position_covariance[8] = alt_std ** 2
        else:  # 'hdop_scaled'
            fix.position_covariance[0] = (self.hdop * lon_std) ** 2
            fix.position_covariance[4] = (self.hdop * lat_std) ** 2
            fix.position_covariance[8] = (2 * self.hdop * alt_std) ** 2

        self.publish('fix', fix)
        self._gga_fresh = False
        return True
