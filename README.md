# rtk_base

## Introduction
ROS 2 driver for an RTK base station receiver. It configures the receiver as a base (survey-in or a
fixed position), forwards the RTCM corrections the receiver outputs to UDP, TCP or serial
outputs, and publishes the position and the raw NMEA so they can be recorded with `ros2 bag`.

- ROS 2 Jazzy, Ubuntu 24.04
- Base receivers: Unicore UM98x (UM982, UM980) configured by the driver (`receiver: unicore`).
  Receivers configured with their own tools, for example a SparkFun RTK Surveyor, use
  `receiver: generic`.
- Rovers read the corrections with `unicore_rtk_driver` (`rtcm.inputs`). The two packages share
  only the RTCM 3 stream.
- The receiver's NMEA can be forwarded to gpsd over UDP (`use_gpsd_udp`), so the base station
  computer sets its clock from GNSS time with chrony and can serve it to the network over NTP
  (see Base station computer setup).

A base computer without ROS can run the relay (`rtk_base_relay`, see RTCM relay without ROS).

## Directory structure
- `rtk_base/base_node.py`: ROS node. Declares parameters, creates the topics, and connects the
  components below.
- `rtk_base/serial_link.py`: receiver serial port. Reader thread, automatic reconnect, serialized
  writes.
- `rtk_base/fix_processor.py`: converts validated GGA and GST sentences into `NavSatFix`.
- `rtk_base/protocol/`: stream framing (NMEA, Unicore ASCII, RTCM 3), checksums (NMEA XOR, Unicore
  CRC-32, RTCM CRC-24Q) and the NMEA parser.
- `rtk_base/receivers/`: receiver specific commands. `unicore.py` for UM98x, `generic.py` for
  receivers configured externally.
- `rtk_base/rtcm/`: RTCM output transports (`transport.py`), routing and statistics (`router.py`),
  RTCM 3 helpers (`rtcm3.py`).
- `rtk_base/tools/relay.py`: RTCM relay for a base station computer without ROS.
- `config/`: example parameter files, relay arguments, and gpsd and chrony examples for the base
  station computer. `launch/`: launch file. `udev/`, `scripts/`: stable device names. `systemd/`:
  node and relay services.

## Installation
```bash
cd ~/ros2_ws/src
git clone https://github.com/GSO-soslab/rtk_base.git
cd ~/ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
colcon build --packages-select rtk_base
```

### Serial port access
The user running the driver must be in the `dialout` group. The installer adds the user to the
group and creates a stable device name from the adapter serial number, or from the USB port when
the adapter has no serial number (CH340 based boards).
```bash
src/rtk_base/scripts/install_udev_rules.sh --dry-run /dev/ttyUSB0 base_um982
src/rtk_base/scripts/install_udev_rules.sh /dev/ttyUSB0 base_um982
```
Log out and back in after the user is added to `dialout`. `udev/99-rtk-gnss.rules` contains
commented examples.

## Quick start
UM982 as a base, corrections served on TCP port 2101 and broadcast on UDP port 2101:
```bash
ros2 launch rtk_base start.launch.py params_file:=$(ros2 pkg prefix rtk_base)/share/rtk_base/config/base_um982.yaml
```
Record the position and the raw NMEA:
```bash
ros2 bag record /base/rtk_base/fix /base/rtk_base/nmea /diagnostics
```
`~/nmea` is only published while something subscribes to it, which includes `ros2 bag record`.

## Receiver configuration
With `configure_receiver: true` the driver sends the following commands every time the serial port
is opened. Nothing is written to receiver flash unless `save_receiver_config: true`.

1. `base.mode_command` if set, otherwise `MODE BASE TIME <base.survey_in_min_duration_s>`, or
   `MODE BASE <lat> <lon> <height>` when `base.use_fixed_position` is true.
2. One `RTCM<number> [<receiver_port>] <period>` per entry of `base.rtcm_messages`.
3. One `<LOG> [<receiver_port>] <period>` per entry of `receiver_logs` (`GPGGA` and `GPGST` by
   default; they produce `~/fix`).
4. Each entry of `receiver_commands`, verbatim.
5. `saveconfig` if `save_receiver_config` is true.

With `configure_receiver: false` (default) only `receiver_commands` are sent, which keeps receivers
that were configured by hand unchanged. The `generic` receiver type never generates commands.
`~/configure_receiver` (`std_srvs/Trigger`) sends the configuration from the current parameters on
demand.

## RTCM outputs
RTCM frames read from the receiver are written to every entry of `rtcm.outputs`. The base station
position from RTCM 1005/1006 is logged and reported in `/diagnostics`.

| URI | Output |
|---|---|
| `udp://HOST:PORT` | One datagram per frame. Broadcast addresses are allowed. |
| `tcpserver://BIND:PORT` | Rovers connect. A client that cannot keep up is disconnected. |
| `tcp://HOST:PORT` | Connect to a relay or rover. Reconnects automatically. |
| `serial:///dev/ttyX?baud=57600` | Radio modem. |

Endpoint options are given as a query string, for example `tcp://192.168.1.10:2101?reconnect_s=0.5`.

| Option | Default | Applies to | Description |
|---|---|---|---|
| `reconnect_s` | 2.0 | tcp | Wait between connection attempts |
| `send_timeout_s` | 0.05 | tcp, tcpserver, serial | Maximum time a send may block |
| `baud` | 57600 | serial | Baud rate |

### SparkFun RTK Surveyor as base
The Surveyor (u-blox ZED-F9P) is configured with its own menu and used with `receiver: generic`
(`config/base_generic.yaml`). The driver forwards the RTCM it outputs and parses its NMEA.

| Surveyor port | Host device | Carries |
|---|---|---|
| Data (4-pin JST, 3.3 V, 115200 baud) | UART, for example `/dev/ttyAMA0` | NMEA from receiver UART1 (internal Serial NMEA Connection switch set to Ext Connector). RTCM after step 2 below. |
| Configure u-blox (USB-C) | `/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00` | NMEA and RTCM from the receiver USB port. Also charges the Surveyor. |
| Configure ESP32 (USB-C) | USB serial adapter, 115200 baud | Surveyor configuration menu. |
| Radio (4-pin JST) | not used | RTCM on receiver UART2, 57600 baud. Pin 1 supplies 3.5 V to 5.5 V; do not connect it to the host. |

Setup:
1. Base type: open the menu on the Configure ESP32 port (any key), Configure Base, and select
   Survey-In. A fixed base with wrong coordinates (for example 0, 0, 0) has no fix in base mode and
   outputs no RTCM 1005. With Survey-In the Surveyor waits for a horizontal accuracy threshold
   (1 m in current firmware), then surveys until the minimum time and mean accuracy are met.
2. RTCM on the Data port: with Surveyor firmware v1.1 the receiver had RTCM output disabled on
   UART1 (`CFG-UART1OUTPROT-RTCM3X = 0`). Enable it through the Configure u-blox port with
   `ubxtool` (package `gpsd-clients`). The setting is stored in the receiver flash; it was kept
   after a power cycle and after the update to firmware v4.4.
   ```bash
   export UBXOPTS="-f /dev/ttyACM0 -s 115200 -P 27"
   ubxtool -g CFG-UART1OUTPROT-RTCM3X
   ubxtool -z CFG-UART1OUTPROT-RTCM3X,1
   ```
3. NMEA messages: `~/fix` is published on GST, which firmware v4.4 does not output
   by default. In the menu select Configure GNSS Messages, Reset to Surveying Defaults (GGA, GSA,
   GST, GSV, RMC). Message rates set in this menu apply to the Data port; for the Configure u-blox
   USB port also select Configure System, Configure RTK operation, Mirror ZED-F9x's UART1 settings
   to USB.
4. Set the SETUP switch to Base. After the survey `/diagnostics` shows `rtcm out bytes/s`, the
   message types (1005, 1074, 1084, 1124 and 1230 are expected) and the base position decoded from
   RTCM 1005.

Base RTCM output was verified with firmware v1.1. With firmware v4.4 the message configuration
above was verified; RTCM output in base mode has not been tested yet.

Firmware updates use the Configure ESP32 port, see the SparkFun RTK firmware documentation. Units
sold before September 2021 have 4 MB flash and can only be updated over USB (GUI or `esptool`).

The Surveyor uses reference station ID 0. Give each base a different ID when more than one base is
in use.

### RTCM relay without ROS
A base station computer that only has to forward corrections does not need ROS. The relay reads
the receiver's serial port and writes the RTCM frames to the same outputs as `rtcm.outputs`, using
the same serial link, framing and transports as the node. It needs Python 3 and pyserial only
(`sudo apt install python3-serial`) and runs from the cloned repository without building it. It
publishes no topics; use the node where the base has to be recorded.

By default the relay does not configure the receiver; set up base mode and RTCM output with the
receiver's own tools. With `--receiver unicore` it sends the UM98x base commands after every
connect (same commands as the node, without the logs).

```bash
git clone https://github.com/GSO-soslab/rtk_base.git
cd rtk_base
python3 -m rtk_base.tools.relay --device /dev/ttyACM0 --baud 115200 \
    --output tcpserver://0.0.0.0:2101 --output udp://192.168.1.255:2101

# UM982: survey-in for 300 s, or a fixed position, and the RTCM messages to enable
python3 -m rtk_base.tools.relay --device /dev/ttyUSB0 --receiver unicore --survey-s 300 \
    --rtcm-message 1006:10 --rtcm-message 1074:1 --output tcpserver://0.0.0.0:2101
python3 -m rtk_base.tools.relay --device /dev/ttyUSB0 --receiver unicore \
    --fixed-position 41.49126464 -71.42186016 -14.11 --output tcpserver://0.0.0.0:2101
```

Arguments can be kept in a file, one option per line (see `config/relay.conf.example`). Lines
starting with `#` and blank lines are skipped. `--name=value` keeps the whole value, spaces
included; `--name v1 v2 v3` gives several values, for example `--fixed-position LAT LON ALT`.
```bash
python3 -m rtk_base.tools.relay @$HOME/rtk/relay.conf
```

The relay prints a status line every `--status-period` seconds (serial state, RTCM rate, message
types, output state, base position from RTCM 1005/1006) and warns when no RTCM arrives for
`--no-rtcm-warning-s` seconds. `systemd/rtk_base_relay.service` starts it at boot; edit the user
and paths in the file before installing it. In a ROS workspace the same tool is available as
`ros2 run rtk_base rtk_base_relay`.

| Option | Default | Description |
|---|---|---|
| `--device` | required | Receiver serial device |
| `--baud` | `115200` | Receiver baud rate |
| `--output` | required | RTCM output URI, repeatable |
| `--receiver` | `generic` | `unicore`: send the UM98x base commands after every connect. `generic`: do not configure the receiver. |
| `--receiver-port` | `''` | Receiver port named in the commands, for example `COM1` |
| `--survey-s` | `120` | Survey-in duration (unicore) |
| `--fixed-position LAT LON ALT` | | Surveyed position instead of survey-in (unicore) |
| `--mode-command` | `''` | Base mode command sent verbatim instead of the generated one |
| `--rtcm-message NUMBER:PERIOD_S` | 1006:10 1033:10 1074:1 1084:1 1094:1 1124:1 | RTCM message to enable, repeatable (unicore) |
| `--save-config` | off | Also save the receiver configuration (writes receiver flash) |
| `--status-period` | `10` | Seconds between status lines, 0 disables them |
| `--serial-reconnect-s` | `1.0` | Wait between attempts to open the serial port |
| `--no-rtcm-warning-s` | `10` | Warn when no RTCM arrives for this long |
| `--log-level` | `INFO` | `DEBUG`, `INFO`, `WARNING` or `ERROR` |

## Base station computer setup
Ubuntu Server 24.04 with ROS 2 Jazzy, for example a Raspberry Pi in the station. The receiver
antenna is outside; only the coaxial cable enters the station. Site specific values (addresses,
user, domain ID, base coordinates) are kept in the station log book.

### Start at boot
`systemd/rtk_base.service` runs `ros2 launch rtk_base start.launch.py` at boot and restarts it if it
stops. Copy the parameter file out of the workspace first, for example
`config/base_generic.yaml` to `~/rtk_base/base.yaml`, and use the receiver's
`/dev/serial/by-id/` name as `device`. Edit the user, `ROS_DOMAIN_ID`, the namespace and the
paths in the service file, then:
```bash
sudo cp systemd/rtk_base.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rtk_base
journalctl -u rtk_base -f
```
Use either this service or the relay service, not both: only one program can read the receiver
port.

### Time from GNSS
The node forwards the receiver's NMEA to gpsd over UDP; chrony takes the time from gpsd and serves
it to the vehicles over NTP. This works without internet.
```bash
sudo apt install --no-install-recommends gpsd gpsd-tools chrony
```
1. gpsd: copy `config/gpsd.example` to `/etc/default/gpsd` and set the UDP port.
   `USBAUTO="false"` stops gpsd from opening the receiver port itself.
2. Node parameters: `use_gpsd_udp: true` and `virtual_port` set to the same port.
3. chrony: copy `config/chrony-rtk_base.conf.example` to `/etc/chrony/conf.d/rtk_base.conf` and set
   the network that may query the time (`allow`).
4. `sudo systemctl restart gpsd chrony`, then check:
```bash
gpsmon                   # gpsd receives the NMEA and has a fix
chronyc sources -v       # GPS line: '*' = the source chrony follows, reach 377 = all recent samples
chronyc tracking         # Reference ID GPS when GNSS time is in use
```
`*` on the GPS line means the computer follows GNSS time. With internet access chrony also lists
the internet servers and may follow one of them instead.

NMEA time arrives a fixed time after the second it describes, and without the receiver's PPS
signal it is accurate to some tens of milliseconds. That is enough to align logs and recordings
between vehicles. To measure the delay, run with internet servers available and compare the GPS
offset with theirs (`chronyc sourcestats`), then set `offset` on the `refclock` line so that GPS
agrees with them. Record the value in the station log book.

Vehicles use the base station computer as their time server, in their own chrony configuration:
```
server <base_ip> iburst
```

## Node: rtk_base_node

### Published topics
Topic names are relative to the node and can be changed with the `*_topic` parameters. They have
the same names, types and conventions as `unicore_rtk_driver`, so base and rover recordings can be
compared.

| Topic | Type | Source | Description |
|---|---|---|---|
| `~/fix` | `sensor_msgs/NavSatFix` | GGA, GST | Position. Published when GST arrives, using the most recent GGA. |
| `~/nmea` | `nmea_msgs/Sentence` | all lines | Raw receiver output. Published only when there is a subscriber. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | | Serial link, fix status, solution type, satellites used, RTCM output rate and message types, output status, base position. |

### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `~/tx_cmd` | `std_msgs/String` | Raw command written to the receiver, followed by `saveconfig` when `save_config_after_tx_cmd` is true. |

### Services

| Service | Type | Description |
|---|---|---|
| `~/configure_receiver` | `std_srvs/Trigger` | Send the receiver configuration built from the current parameter values, independent of `configure_receiver`. |

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `device` | `/dev/ttyUSB0` | Receiver serial device |
| `baudrate` | `115200` | Receiver baud rate |
| `serial_reconnect_s` | `1.0` | Wait between attempts to open the serial port |
| `gps_frame_id` | `rtk_base` | Frame of `fix` and `nmea` |
| `use_gnss_time` | `false` | Stamp `fix` with GNSS time instead of arrival time |
| `fix_covariance_mode` | `hdop_scaled` | `hdop_scaled`: variance = (HDOP x GST std)^2, height uses 2 x HDOP. `gst`: variance = GST std^2. |
| `strict_validation` | `false` | Publish `fix` only after a new valid GGA |
| `epe_quality0` ... `epe_quality9` | | Default position error per GGA fix quality, kept for compatibility |
| `use_gpsd_udp` | `false` | Forward NMEA lines to gpsd over UDP |
| `gpsd_udp_host` | `127.0.0.1` | gpsd address |
| `virtual_port` | `5005` | gpsd UDP port (gpsd: `DEVICES="udp://127.0.0.1:<port>"`) |
| `gpsd_udp_patterns` | `['$GN']` | Lines containing any of these strings are forwarded to gpsd |
| `receiver` | `unicore` | `unicore` or `generic` |
| `receiver_port` | `''` | Receiver port named in output commands, for example `COM1`. Empty uses the port the command is received on. |
| `configure_receiver` | `false` | Send the receiver configuration each time the port is opened |
| `save_receiver_config` | `false` | Append `saveconfig` to the configuration |
| `receiver_logs` | `['GPGGA:1', 'GPGST:1']` | Outputs to enable, `LOG:period_s` |
| `receiver_commands` | `['']` | Additional raw commands sent after the configuration |
| `save_config_after_tx_cmd` | `true` | Send `saveconfig` after each `~/tx_cmd` command |
| `base.mode_command` | `''` | Base mode command sent verbatim. Empty generates it from the parameters below. |
| `base.survey_in_min_duration_s` | `120` | Survey-in duration |
| `base.use_fixed_position` | `false` | Use the fixed position instead of survey-in |
| `base.fixed_latitude_deg` | `0.0` | Fixed base latitude |
| `base.fixed_longitude_deg` | `0.0` | Fixed base longitude |
| `base.fixed_altitude_m` | `0.0` | Fixed base height, in the reference expected by the receiver |
| `base.rtcm_messages` | `['1006:10', '1033:10', '1074:1', '1084:1', '1094:1', '1124:1']` | RTCM messages to enable, `number:period_s` |
| `rtcm.outputs` | `['']` | RTCM outputs, see RTCM outputs |
| `rtcm.correction_timeout_s` | `5.0` | Time without RTCM from the receiver after which `/diagnostics` reports a warning |
| `diagnostics_period` | `1.0` | Period of `/diagnostics` in seconds |
| `debug` | `false` | Log every sentence at debug level |

Topic name parameters: `fix_topic`, `nmea_topic`, `tx_cmd_topic`.

## Shared code
These modules are copies of the same files in `unicore_rtk_driver`, so that each package can be
installed and maintained on its own: `protocol/checksums.py`, `protocol/framer.py`,
`protocol/nmea_parser.py`, `serial_link.py` and `rtcm/rtcm3.py`. `fix_processor.py` is derived
from `processor.py` there. A fix in one of these files belongs in both packages.

## Testing
```bash
colcon test --packages-select rtk_base
colcon test-result --verbose
```
Without ROS, the tests that do not need ROS messages run with `python3 -m pytest` from the
repository root (needs `pytest` and `pyserial`); `test/test_fix.py` needs the ROS message packages.

## Known limitations
The following have not been verified on hardware yet:
- UM982 base commands: `MODE BASE TIME`, `MODE BASE <lat> <lon> <height>` (height reference to be
  confirmed for the firmware in use) and `RTCM<number> [port] <period>`.
- SparkFun RTK Surveyor RTCM output in base mode with firmware v4.4.

## Acknowledgment
`rtk_base/protocol/nmea_parser.py` is derived from
[nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver) (BSD license, see the header
of the file). The serial link, framing, RTCM transports and the fix conversion come
from `unicore_rtk_driver`.
