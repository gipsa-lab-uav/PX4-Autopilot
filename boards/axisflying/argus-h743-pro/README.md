# AxisFlying Argus H743 Pro — PX4 development target

This target is for the **4–8S, dual ICM42688P** Argus H743 Pro supplied by
AxisFlying. It uses this checkout's PX4 and NuttX versions. It is an experimental
board port: successful compilation is not proof of hardware or flight operation.
Hardware validation is incomplete, and it has not been flight tested. This is
not manufacturer-supported PX4 firmware.

Hardware references:

- [AxisFlying product specification](https://www.axisflying.com/products/h743-pro).
- [Betaflight AXISFLYINGH743PRO pin map](https://github.com/betaflight/config/blob/master/configs/AXFL/AXISFLYINGH743PRO/config.h).
- [Puya PY25Q128HA datasheet](https://www.puyasemi.com/download_path/%E6%95%B0%E6%8D%AE%E6%89%8B%E5%86%8C/Flash/PY25Q128HA_Datasheet_V2.0.pdf).

The pin map is used as hardware data; the board implementation uses existing PX4
code and drivers. Later boards with BMI270/LSM6DSK320X gyros, H743 Mini boards,
and 12S revisions are outside this target's verified specification.

## Build

From the repository root with the PX4 build dependencies installed:

```sh
make axisflying_argus-h743-pro_bootloader
make axisflying_argus-h743-pro_default
```

Build the bootloader first. PX4 copies its binary into this board's `extras/`
folder automatically, so the application includes the matching `bl_update` image.
When using the existing Python environment in this checkout, add
`PYTHON_EXECUTABLE="$PWD/.venv/bin/python"` to each command.

Artifacts:

- Bootloader: `build/axisflying_argus-h743-pro_bootloader/axisflying_argus-h743-pro_bootloader.bin`.
- Firmware package: `build/axisflying_argus-h743-pro_default/axisflying_argus-h743-pro_default.px4`.
- Raw firmware: `build/axisflying_argus-h743-pro_default/axisflying_argus-h743-pro_default.bin`.

Both targets are discovered automatically by `Tools/ci/generate_board_targets_json.py`
and the existing all-targets build workflow.

## Flash layout and identity

| Area | Address | Size |
| --- | --- | --- |
| PX4 bootloader | `0x08000000` | 128 KiB, sector 0 |
| PX4 application | `0x08020000` | 1792 KiB, sectors 1–14 |
| Persistent parameters | `0x081E0000` | 128 KiB, sector 15 |
| External flash | SPI3, PD7 chip select | 16 MiB, LittleFS |

Firmware updates through the PX4 bootloader leave the parameter sector untouched.
A complete STM32 mass erase does not preserve parameters.

The board ID **60001** is a provisional local development ID, unique in this
checkout, and is identical in `firmware.prototype` and `src/hw_config.h`.
It has not been reserved upstream. Do not reuse a Kakute/Matek board ID or use
their binaries for this board.

USB uses the [pid.codes development test identity `1209:0001`](https://pid.codes/1209/0001/),
with distinct application and bootloader product strings. This is not an
allocated AxisFlying production identity. Reserve a board ID and a production
USB VID/PID before distributing a production target. Product strings are
`PX4 FMU Argus H743 Pro` and `PX4 FMU Argus H743 Pro BL`: both match
[QGroundControl's `^PX4 FMU` description fallback](https://github.com/mavlink/qgroundcontrol/blob/v5.0.8/src/Comms/USBBoardInfo.json),
and the `BL` suffix identifies the bootloader. Select the serial port explicitly
if uploader autodetection does not recognize the test VID/PID.

No VBUS-sense wiring is specified by the hardware pin map. The application
registers USB CDC unconditionally, and the bootloader waits three seconds for
USB on every normal reset. PA8 is a camera switch, and is never used as a USB
detector. There is no reliable USB/battery source-valid detection.

## Initial installation

Back up the existing firmware/configuration and blackbox logs first. Remove
propellers and disconnect ESC/motor power for initial bring-up.

1. Hold BOOT while connecting USB to enter the STM32 ROM DFU bootloader.
2. Use STM32CubeProgrammer to write the **bootloader binary** at `0x08000000`.
   Verify the write, release BOOT, and power-cycle the board. Do not write the
   raw application at `0x08000000`.
3. Upload the **`.px4` firmware package** through the PX4 bootloader. Select the
   custom firmware file in QGroundControl, or use this checkout's uploader:

   ```sh
   .venv/bin/python Tools/px4_uploader.py --port /dev/ttyACM0 \
       build/axisflying_argus-h743-pro_default/axisflying_argus-h743-pro_default.px4
   ```

   Adjust the port for the actual device. Do not override a board-ID mismatch.
4. Open the PX4 MAVLink console and check `ver all` and the sensor commands
   listed below before connecting peripherals.

This change creates build artifacts; it does not flash attached hardware.

### USB connection troubleshooting

An upload that completes successfully confirms communication with the
bootloader, not successful application startup. The bootloader normally appears
for three seconds, disconnects, and is replaced by the application USB device
after PX4 startup. Wait at least ten seconds after reconnecting without BOOT.

On Linux, close QGroundControl and monitor the transition with `sudo dmesg -w`,
then inspect `lsusb` and `ls -l /dev/ttyACM*`. A stable `1209:0001` device with
the application product string should be available for MAVLink. If
QGroundControl does not automatically connect, enable Pixhawk USB auto-connect
or create a Serial link in Settings > Comm Links for the application port at
115200 baud, with flow control disabled. The port number may change when the
bootloader disconnects.

If USB never returns after the bootloader, or repeatedly disconnects, collect
the kernel log and check the UART3 NSH startup console at 57600 baud (PD8/TX,
PD9/RX, common ground, 3.3 V serial adapter). This indicates an enumeration or
application-startup problem that changing QGroundControl settings cannot fix.
The initial `PX4 Argus H743 Pro` product name did not match QGroundControl's
PX4 description fallback; reupload the rebuilt application if using that image.

## Sensor and peripheral map

| Function | Bus / STM32 pins | PX4 configuration |
| --- | --- | --- |
| IMU1, ICM42688P | SPI1 PA5/PA6/PA7; CS PC4; DRDY PA4 | `icm42688p -s -b 1 -R 0 -C 32000` |
| IMU2, ICM42688P | SPI4 PE12/PE13/PE14; CS PE15; DRDY PE11 | `icm42688p -s -b 4 -R 2 -C 32000` |
| Gyro clocks | PB0/TIM3_CH3, PB1/TIM3_CH4 | two 32 kHz outputs from TIM3 |
| DPS368 barometer | I2C2 PB10/PB11 | `dps310 -I -b 2 -8`, addresses `0x76`/`0x77` |
| External I2C | I2C1 PB8/PB9 | external compass/airspeed support |
| AT7456E analog OSD | SPI2 PB13/PB14/PB15; CS PB12 | set `OSD_ATXXXX_CFG`, reboot |
| PY25Q128HA NOR | SPI3 PB3/PB4/PB5; CS PD7 | JEDEC `85 20 18`; LittleFS on `/fs/flash` |
| Battery voltage ADC | PC0 / ADC1 channel 10 | calibrate `BAT1_V_DIV` |
| ESC current ADC | PC1 / ADC1 channel 11 | calibrate `BAT1_A_PER_V`, `BAT_V_OFFS_CURR` |
| Status LED | PD3 | active low |
| Active buzzer | PC13 | GPIO output; startup tunes disabled by default |
| VTX power switch | PE2 | active low; off at startup |
| Camera selection | PA8 | camera 1 at startup |

The HSE is configured for 8 MHz, consistent with the Betaflight target's default,
and the CPU runs at 480 MHz. Confirm the oscillator and LED polarity during bench
bring-up. IMU rotations derive from Betaflight CW0/CW90 and PX4's sensor frame
conversion; confirm both IMUs produce the same signs on all axes before flight.

There is no onboard magnetometer. `SYS_HAS_MAG=0` and gravity fusion are defaults.
Configure external compass drivers and `SYS_HAS_MAG` if fitting a compass.
Voltage-divider and current-sensor calibration are deliberately not copied from
another board. Until calibrated, the ADC scaling defaults are zero: battery
telemetry will not be usable. Set the battery cell count and chemistry as well.

The PE5 LED-strip pad and additional PD12–PD15 servo pads are not configured as
outputs in this eight-motor target. The onboard status LED is supported.

## Serial ports

NuttX preserves hardware UART order; USART6 is disabled because PC6/PC7 are M1/M2.
UART3 is the debug console, avoiding unsolicited console text on the RC link.

| Board port | TX / RX | Device | Default role |
| --- | --- | --- | --- |
| UART1 | PB6 / PB7 | `/dev/ttyS0` | TELEM1 |
| UART2 | PD5 / PD6 | `/dev/ttyS1` | RC |
| UART3 | PD8 / PD9 | `/dev/ttyS2` | NSH debug console, 57600 baud |
| UART4 | PC10 / PC11 | `/dev/ttyS3` | TELEM2 |
| UART5 | RX PD2 only | `/dev/ttyS4` | ESC telemetry |
| UART7 | PE8 / PE7 | `/dev/ttyS5` | TELEM3 |
| UART8 | PE1 / PE0 | `/dev/ttyS6` | GPS1 |

The legacy RC scanner is included for the default UART2 receiver input, as are
the separate CRSF, SBUS, DSM and GHST drivers. If choosing a separate driver,
disable the scanner's `RC_PORT_CONFIG` first to avoid two drivers opening the
same UART. Confirm receiver inversion and failsafe with the actual receiver.
UART5 has no exposed TX; do not assign it a bidirectional peripheral.

## Motor outputs

PWM and DShot use the physical output order below. Assign actuator functions in
PX4 for the selected airframe; Betaflight motor numbering is not an airframe map.

| Output | Pin | Timer |
| --- | --- | --- |
| M1 | PC6 | TIM8_CH1 |
| M2 | PC7 | TIM8_CH2 |
| M3 | PC8 | TIM8_CH3 |
| M4 | PC9 | TIM8_CH4 |
| M5 | PA0 | TIM2_CH1 |
| M6 | PA1 | TIM2_CH2 |
| M7 | PA2 | TIM2_CH3 |
| M8 | PA3 | TIM2_CH4 |

Outputs in each four-channel timer group share a PWM rate/protocol. TIM5 is
reserved for the high-resolution timer; TIM3 is reserved for gyro clocks.
The bootloader and application leave motor pins as inputs before output drivers
start. Check pulse/DShot behavior and motor order with propellers removed.

## External flash setup and logging

The existing NuttX M25P driver is configured with Puya manufacturer `0x85`,
MT25Q-compatible type `0x20`, 256-byte pages, 4 KiB erases and 20 MHz SPI.
`M25P_MEMORY_TYPE=0xff` ensures the incompatible M25P128 256 KiB erase geometry
is not selected. There are no changes to the NuttX submodule's tracked files.
Only the documented Puya part is enabled; verify the actual fitted chip if
initialization reports an unrecognized device.

The board only attempts to **mount** LittleFS. It never formats flash after a
mount failure, preserving existing Betaflight/INAV logs and damaged filesystems
for recovery. On the first installation, after saving previous blackbox logs,
format explicitly from the MAVLink console:

```sh
mklittlefs /dev/mtd0 /fs/flash
mkdir /fs/flash/etc
reboot
```

This erases all external flash contents. It does not erase internal parameters.
After reboot, check `mount`, `df`, `logger status` and `/fs/flash/log`.
`SDLOG_MAX_SIZE=2` MiB and `SDLOG_ROTATE=90` limit the small log store. Logging
rates and flash latency still require hardware testing. Mission storage defaults
to RAM (`SYS_DM_BACKEND=1`), so missions do not survive a reboot.

## Bench validation required

Use the MAVLink console to check:

```sh
ver all
icm42688p status
dps310 -I -b 2 -8 status
listener sensor_accel 4
listener sensor_gyro 4
listener sensor_baro
listener battery_status
logger status
```

Confirm two distinct accelerometers and gyroscopes, consistent axis signs and
approximately -9.81 m/s² Z acceleration while the board is level. Confirm
reasonable barometric pressure, measured battery/current values, GPS operation,
RC/failsafe, motor output order, OSD, log download and USB reconnects. Verify
parameter persistence across a power-cycle and a PX4 firmware upload. Inspect
both PB0/PB1 clocks and all motor signals with a scope before flight testing.

Upstream release also requires allocated board/USB identities and flight-test
logs. A successful build alone does not satisfy those requirements.

## Validation of this port

On 2026-10-07, based on checkout commit `b18a9dcc2f`, both targets built
successfully with GNU Make, this checkout's Python environment and ARM GCC toolchain:

- Bootloader: 39,428 bytes within its 128 KiB reservation.
- Firmware: 1,755,176 bytes within its 1,835,008-byte application region
  (95.65% used; 79,832 bytes remain).
- The `.px4` image payloads match their raw binaries. ELF vector addresses,
  reset entry points, the compiled bootloader board ID, required drivers and
  the bundled bootloader were checked. Both targets are present in CI discovery.
- All new C/C++ files pass the repository's AStyle checks; startup scripts pass
  shell syntax checks.
- Both compiled USB product strings match the description rules published by
  QGroundControl 4.4.3, 5.0.8 and current source. The previous strings did not.
- A user reported successful DFU bootloader programming and PX4 firmware upload.
  Their Linux USB log confirmed the bootloader-to-application handoff after
  about three seconds and application CDC/ACM enumeration on `ttyACM0`.
- A host simulation exercised the actual NuttX M25P driver using this target's
  generated flash configuration. It confirmed Puya JEDEC recognition, 16 MiB
  capacity, 256-byte pages, 4 KiB erase geometry, read/write, writes crossing
  page boundaries, neighboring-sector preservation and rejection of an
  unsupported manufacturer. Address/undefined-behavior sanitizers were enabled.

The build and host checks do not verify electrical wiring, real flash timing,
sensor signs, USB reconnect reliability or flight behavior. Complete the bench
validation above on the actual controller.
