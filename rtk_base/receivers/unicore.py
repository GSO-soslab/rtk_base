"""Unicore UM98x (UM982 / UM980) receivers.

Command syntax follows the Unicore N4 "Commands and Logs" reference:
    MODE BASE TIME <seconds>                       survey-in
    MODE BASE <lat_deg> <lon_deg> <height_m>       fixed base
    RTCM<number> [<port>] <period_s>               enable an RTCM output
    <LOG> [<port>] <period_s>                      enable a log, e.g. GPGGA 1
    saveconfig
Replies look like:  $command,MODE BASE TIME 120,response: OK*xx
"""
from typing import List

from rtk_base.receivers.base import BaseStationSettings, GnssReceiver, LogRequest

# What the base node consumes: GPGGA + GPGST -> ~/fix.
DEFAULT_LOGS = [LogRequest('GPGGA', 1.0), LogRequest('GPGST', 1.0)]


class UnicoreReceiver(GnssReceiver):
    name = 'unicore'

    def configuration_commands(self, base: BaseStationSettings,
                               logs: List[LogRequest]) -> List[str]:
        port = f'{self.port_name} ' if self.port_name else ''
        if base.mode_command:
            cmds = [base.mode_command]
        elif base.use_fixed_position:
            cmds = [f'MODE BASE {base.fixed_latitude_deg:.10f} '
                    f'{base.fixed_longitude_deg:.10f} {base.fixed_altitude_m:.4f}']
        else:
            # The UM98x survey-in is time based; the accuracy target is not used.
            cmds = [f'MODE BASE TIME {int(base.survey_in_min_duration_s)}']
        for msg in base.rtcm_messages:
            cmds.append(f'RTCM{msg.number} {port}{msg.period_s:g}')
        for log in logs:
            cmds.append(f'{log.name} {port}{log.period_s:g}')
        return cmds

    def save_configuration_command(self) -> str:
        return 'saveconfig'

    def is_command_response(self, line: str) -> bool:
        return line.startswith('$command,')
