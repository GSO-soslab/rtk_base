from typing import List

from rtk_base.receivers.base import (BaseStationSettings, GnssReceiver, LogRequest, RtcmMessage,
                                     parse_log_list, parse_rtcm_message_list)
from rtk_base.receivers.generic import GenericReceiver
from rtk_base.receivers.unicore import UnicoreReceiver

RECEIVERS = {cls.name: cls for cls in (UnicoreReceiver, GenericReceiver)}


def create_receiver(name: str, port_name: str = '') -> GnssReceiver:
    try:
        return RECEIVERS[name](port_name)
    except KeyError:
        raise ValueError(f'unknown receiver {name!r}, expected one of {sorted(RECEIVERS)}')


def build_commands(receiver: GnssReceiver, base: BaseStationSettings, logs: List[LogRequest],
                   extra_commands: List[str] = (), configure: bool = True,
                   save: bool = False) -> List[str]:
    """Commands to send after each (re)connect: base mode, RTCM messages and
    logs if `configure`, then `extra_commands`, then the save command if
    `save` and anything was sent."""
    cmds = list(receiver.configuration_commands(base, logs)) if configure else []
    cmds += [c for c in extra_commands if c and c.strip()]
    save_cmd = receiver.save_configuration_command()
    if cmds and save and save_cmd:
        cmds.append(save_cmd)
    return cmds


__all__ = ['BaseStationSettings', 'GnssReceiver', 'LogRequest', 'RtcmMessage', 'parse_log_list',
           'parse_rtcm_message_list', 'GenericReceiver', 'UnicoreReceiver', 'RECEIVERS',
           'create_receiver', 'build_commands']
