"""NMEA -> NavSatFix (needs ROS message packages)."""
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import NavSatFix, NavSatStatus

from rtk_base.fix_processor import Config, FixProcessor
from rtk_base.protocol.checksums import nmea_checksum


class Log:
    def __init__(self):
        self.warnings = []

    def debug(self, *a, **k):
        pass

    info = debug

    def warn(self, msg, **k):
        self.warnings.append(msg)

    warning = error = warn


def nmea(body):
    return f'${body}*{nmea_checksum(body):02X}'


def gga(quality, satellites=24, hdop=0.6):
    return nmea(f'GNGGA,173144.00,4129.47587847,N,07125.31160943,W,{quality},{satellites},'
                f'{hdop},16.2800,M,-30.3918,M,1.0,0001')


# lat std 0.010, lon std 0.011, alt std 0.020
GST = nmea('GNGST,173144.00,1.49,0.012,0.009,156.0,0.010,0.011,0.020')
STAMP = Time(sec=1763659904, nanosec=0)


def run(lines, **cfg):
    out, log = [], Log()
    proc = FixProcessor(Config(**cfg), lambda name, msg: out.append((name, msg)), log)
    results = [proc.handle_nmea(line, STAMP) for line in lines]
    return proc, out, results, log


def test_position_and_frame():
    _, out, _, _ = run([gga(4), GST], gps_frame_id='base/gps')
    assert [name for name, _ in out] == ['fix']
    fix = out[0][1]
    assert fix.header.frame_id == 'base/gps'
    assert (fix.header.stamp.sec, fix.header.stamp.nanosec) == (1763659904, 0)
    assert fix.latitude == pytest.approx(41 + 29.47587847 / 60)
    assert fix.longitude == pytest.approx(-(71 + 25.31160943 / 60))
    # ellipsoidal height = MSL height + geoid separation
    assert fix.altitude == pytest.approx(16.28 - 30.3918)


@pytest.mark.parametrize('quality, status', [
    (4, NavSatStatus.STATUS_GBAS_FIX),
    (5, NavSatStatus.STATUS_GBAS_FIX),
    (2, NavSatStatus.STATUS_SBAS_FIX),
    (1, NavSatStatus.STATUS_FIX),
    (0, NavSatStatus.STATUS_NO_FIX),
    (7, NavSatStatus.STATUS_NO_FIX),     # manual position is not in the table: no fix
])
def test_status_from_gga_quality(quality, status):
    proc, out, _, _ = run([gga(quality), GST])
    assert out[0][1].status.status == status
    assert proc.gga_quality == quality and proc.num_satellites == 24


def test_covariance_modes():
    _, out, _, _ = run([gga(4), GST], fix_covariance_mode='gst')
    cov = out[0][1].position_covariance
    assert (cov[0], cov[4], cov[8]) == pytest.approx((0.011 ** 2, 0.010 ** 2, 0.020 ** 2))
    assert out[0][1].position_covariance_type == NavSatFix.COVARIANCE_TYPE_APPROXIMATED

    _, out, _, _ = run([gga(4, hdop=0.6), GST])      # default: hdop scaled
    cov = out[0][1].position_covariance
    assert (cov[0], cov[4], cov[8]) == pytest.approx(
        ((0.6 * 0.011) ** 2, (0.6 * 0.010) ** 2, (2 * 0.6 * 0.020) ** 2))
    with pytest.raises(ValueError):
        FixProcessor(Config(fix_covariance_mode='bogus'), lambda *a: None, Log())


def test_fix_is_published_on_gst_only():
    _, out, _, _ = run([gga(4)])
    assert out == []


def test_strict_validation_needs_a_fresh_gga_per_fix():
    _, out, results, _ = run([gga(4), GST, GST], strict_validation=True)
    assert len(out) == 1 and results[-1] is False
    _, out, _, _ = run([gga(4), GST, GST])           # default: second GST repeats the last position
    assert len(out) == 2


def test_bad_checksum_and_other_sentences():
    bad = gga(4).replace('4129', '4130')
    proc, out, results, log = run([bad, GST, nmea('GNVTG,1,T,,M,0.1,N,0.2,K,A')])
    assert results == [False, True, True]
    assert 'invalid checksum' in log.warnings[0]
    assert proc.gga_quality is None             # the corrupted GGA was not used


def test_gnss_time_stamp():
    _, out, _, _ = run([gga(4), GST], use_gnss_time=True)
    assert out[0][1].header.stamp.sec % 86400 == 17 * 3600 + 31 * 60 + 44
