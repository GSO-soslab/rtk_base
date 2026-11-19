"""Receiver type for modules configured with their own tools.

For example a SparkFun RTK Surveyor (u-blox ZED-F9P) set up as a base through
its own menu.  The node parses its NMEA and forwards its RTCM.  This receiver
type generates no configuration commands; extra commands are sent as given.
"""
from typing import List

from rtk_base.receivers.base import BaseStationSettings, GnssReceiver, LogRequest


class GenericReceiver(GnssReceiver):
    name = 'generic'

    def configuration_commands(self, base: BaseStationSettings,
                               logs: List[LogRequest]) -> List[str]:
        return []
