# iFlight BLITZ Wing H743 — PX4 development target

Experimental PX4 firmware and bootloader for the iFlight BLITZ Wing H743
(reference supplied: BF15391), STM32H743, single ICM42688P and DPS310 revision.
This uses the PX4/NuttX versions in this checkout. Compilation does not establish
hardware or flight operation. Bench and flight validation remain outstanding.

Hardware data was cross-checked against:

- [iFlight product specification](https://shop.iflight.com/index.php?product_id=2174&route=product%2Fproduct).
- [ArduPilot Wing definition](https://github.com/ArduPilot/ardupilot/blob/3b52469e50bb5b415b16d9e30f22fb19ba6cb8d4/libraries/AP_HAL_ChibiOS/hwdef/BlitzWingH743/hwdef.dat)
  and its [parent definition](https://github.com/ArduPilot/ardupilot/blob/3b52469e50bb5b415b16d9e30f22fb19ba6cb8d4/libraries/AP_HAL_ChibiOS/hwdef/BlitzH743Pro/hwdef.dat).
- [INAV shared Pro/Wing pin map](https://github.com/iNavFlight/inav/blob/940b9281bbefb06b85d442ac5b456c81ae8672c9/src/main/target/IFLIGHT_BLITZ_H7_PRO/target.h)
  and [output map](https://github.com/iNavFlight/inav/blob/940b9281bbefb06b85d442ac5b456c81ae8672c9/src/main/target/IFLIGHT_BLITZ_H7_PRO/target.c).

Only pin/clock/orientation data is taken from those definitions; the implementation
uses this checkout's PX4 drivers and existing BSD-licensed board support.
iFlight lists later SPA06-001 and SPL06-003 barometer revisions. This target
starts DPS310 only: check the fitted sensor before using it on a later revision.
The BLITZ H7 Pro multirotor board is a different target.

## Build and upload

From the repository root, with the existing Python environment activated:

```sh
. .venv/bin/activate
make iflight_blitz-wing-h743_bootloader
make iflight_blitz-wing-h743_default
```

The bootloader build copies its binary into `extras/` for the application's
`bl_update` command. Both targets are discovered by the existing PX4 CI tooling.
GNU Make and Ninja are supported; there is no dependency on a temporary Ninja path.

Artifacts:

- `build/iflight_blitz-wing-h743_bootloader/iflight_blitz-wing-h743_bootloader.bin`
- `build/iflight_blitz-wing-h743_default/iflight_blitz-wing-h743_default.px4`
- `build/iflight_blitz-wing-h743_default/iflight_blitz-wing-h743_default.bin`

Back up the existing firmware, configuration and SD logs. Remove propellers
and disconnect ESC/servo power for initial bring-up.

1. Hold BOOT while connecting USB to enter STM32 ROM DFU. Release BOOT.
2. Confirm `sudo dfu-util -l` lists `0483:df11`, alternate setting 0, Internal Flash.
3. Write the bootloader at `0x08000000`:

   ```sh
   sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave \
       -D build/iflight_blitz-wing-h743_bootloader/iflight_blitz-wing-h743_bootloader.bin
   ```

4. Reconnect without BOOT. Close QGroundControl while uploading the application:

   ```sh
   make iflight_blitz-wing-h743_default upload
   ```

   Or select the port explicitly:

   ```sh
   .venv/bin/python Tools/px4_uploader.py --port /dev/ttyACM0 \
       build/iflight_blitz-wing-h743_default/iflight_blitz-wing-h743_default.px4
   ```

   Adjust the port as needed. Do not override a board-ID mismatch.
5. Reconnect, allow PX4 time to start, and open QGroundControl. Enable Pixhawk USB
   auto-connect. If necessary, create a Serial link to the application port at
   115200 baud with flow control disabled.

These instructions install PX4 on this board; the binaries are incompatible
with ArduPilot's flash layout and with the Argus target. Creating this port
and building it does not flash attached hardware.

## Flash layout and USB identity

| Area | Address | Size |
| --- | --- | --- |
| Bootloader | `0x08000000` | 128 KiB, sector 0 |
| Application | `0x08020000` | 1792 KiB, sectors 1–14 |
| Persistent parameters | `0x081E0000` | 128 KiB, sector 15 |
| Logs and optional mission storage | microSD | FAT32 |

The provisional PX4 board ID is **60002**, matching the bootloader and firmware
package. It is unique in this checkout but is not reserved upstream. The Argus
port uses 60001; its binaries must not be flashed to the BLITZ Wing.
Bootloader firmware uploads preserve the parameter sector. A mass erase does not.

USB uses the public pid.codes development identity `1209:0001`, not an allocated
iFlight production identity. Product strings are `PX4 FMU iFlight Blitz Wing H743`
and `PX4 FMU iFlight Blitz Wing H743 BL Bootloader`. They match QGroundControl's
`^PX4 FMU` description fallback. The mixed-case `Blitz` is deliberate: QGC 4.4.3
and 5.0.8 [treat any uppercase `BL` substring as a bootloader](https://github.com/mavlink/qgroundcontrol/blob/v5.0.8/src/Comms/QGCSerialPortInfo.cc#L244-L253),
so an uppercase `BLITZ` application name would prevent automatic connection.
The bootloader retains explicit `BL` and `Bootloader` markers. Production release
requires reserved board and USB identities.

USB uses PA11/PA12 without a verified VBUS detector. PA9 is UART1 TX and must
never be treated as a VBUS input. The bootloader waits three seconds on each
normal reset, disconnects, and the application later enumerates. That single
USB disconnect is expected; repeated disconnects require startup diagnostics.

## Sensors, storage and peripherals

| Function | Bus / pins | PX4 setup |
| --- | --- | --- |
| ICM42688P | SPI1: PA5 SCK, PA6 MISO, PD7 MOSI; CS PC15 | `icm42688p -s -b 1 -R 0 start` |
| DPS310 | I2C2: PB10 SCL, PB11 SDA | probe `0x76`, then `0x77` |
| External I2C1 | PB6 SCL, PB7 SDA | compass/airspeed drivers |
| External I2C2 | PB10 SCL, PB11 SDA, shared with barometer | avoid address conflicts |
| AT7456E OSD | SPI2: PB13 SCK, PB14 MISO, PB15 MOSI; CS PB12 | enable `OSD_ATXXXX_CFG`, reboot |
| microSD | SDMMC1: PC8–PC11 data, PC12 clock, PD2 command | four-bit IDMA, `/fs/microsd` |
| CAN | FDCAN1: PD0 RX, PD1 TX; PD3 silent control low | DroneCAN, `UAVCAN_ENABLE` |
| Battery voltage | PC0, ADC1 input 10 | nominal `BAT1_V_DIV=11` |
| Battery current | PC1, ADC1 input 11 | nominal `BAT1_A_PER_V=50` |
| Analog airspeed | PC4, ADC1 input 4 | `ADC_DP_V_DIV=2`, configure `SENS_DPRES_ANSC` |
| Status LEDs | PE3 / PE4 | active low; confirm polarity on hardware |
| Buzzer | PA15, TIM2 channel 1 | tone alarm |
| VTX power | PD10 | high enables power; off at startup |
| Camera selection | PD11 | low at startup; confirm selected input |

The external oscillator is 8 MHz and CPU clock is 480 MHz. The IMU uses its
internal clock and timed FIFO polling because no interrupt pin is specified in
the compared upstream definitions. PX4's driver already flips sensor Y/Z,
so rotation 0 produces the same conversion as ArduPilot's raw-sample roll 180.
Confirm axis signs on the actual board before flight.

There is no onboard compass. `SYS_HAS_MAG=0` and gravity fusion are defaults;
configure an external compass before using an airframe that requires one.
Battery scaling uses the Wing-specific ArduPilot reference, rather than the
Pro board's different divider. Verify voltage/current with a meter and configure
battery chemistry/cell count. Analog airspeed remains disabled until calibrated.

VTX/camera pins have defined startup states. There is no automatic transmitter
switching integration. The PA8 LED-strip pad is reserved and not driven by this
port. Servo BEC voltage is selected on the hardware; software output configuration
does not set that voltage.

## Serial ports

INAV's count of eight serial ports includes USB plus seven hardware UARTs.
UART5 is disabled because its usual pins belong to SDMMC1. NuttX preserves the
hardware order, including USART6. UART3 is the NSH debug console at 57600 baud.

| Board port | TX / RX | Device | Default role |
| --- | --- | --- | --- |
| UART1 | PA9 / PA10 | `/dev/ttyS0` | TELEM1 / digital VTX configuration |
| UART2 | PD5 / PD6 | `/dev/ttyS1` | RC |
| UART3 | PD8 / PD9 | `/dev/ttyS2` | NSH console |
| UART4 | PB9 / PB8 | `/dev/ttyS3` | GPS1 |
| UART6 | PC6 / PC7 | `/dev/ttyS4` | TELEM2 |
| UART7 | PE8 / PE7 | `/dev/ttyS5` | TELEM3 |
| UART8 | PE1 / PE0 | `/dev/ttyS6` | ESC telemetry |

The RC scanner defaults to UART2, consistent with the Wing ArduPilot definition.
For CRSF/SBUS/DSM/GHST separate drivers, disable `RC_PORT_CONFIG` first to avoid
sharing a port with the scanner. Confirm receiver inversion and failsafe.
UART1's TELEM1 role is not an automatic MSP DisplayPort configuration.

## Actuator outputs

| Outputs | Pins | Timer | Supported protocols |
| --- | --- | --- | --- |
| S1–S2 | PB0, PB1 | TIM3 channels 3–4 | PWM / DShot |
| S3–S6 | PA0–PA3 | TIM5 channels 1–4 | PWM / DShot |
| S7–S10 | PD12–PD15 | TIM4 channels 1–4 | PWM / DShot |
| S11–S12 | PE5, PE6 | TIM15 channels 1–2 | PWM |

All 12 outputs use physical connector order. Assign their functions for the chosen
PX4 fixed-wing, VTOL or other airframe; there are no default motor assignments.
Channels on a timer share rate/protocol. TIM15 has no update DMA on STM32H7;
S11/S12 support PWM servos, not DShot. S10 has no TIM4_CH4 DMAMUX capture mapping,
so DShot output works but bidirectional DShot telemetry does not. TIM8 is reserved
for PX4 high-resolution timing, TIM2 for the buzzer, TIM7 for DroneCAN. Both the
bootloader and application leave S1–S12 as inputs until output drivers start.

## SD card and mission storage

Use a backed-up FAT32 microSD card, inserted before power-up. No card-detect pin
is published, so hot-plug is not supported. The boot sequence attempts to mount
existing storage and **never formats automatically** on a mount failure.
A missing card must not prevent USB/firmware startup; logging requires a card.

Persistent parameters are in internal flash regardless of SD card presence.
Mission storage defaults to the SD file backend when storage mounts, and RAM
otherwise. Explicit user parameters take precedence; set `SYS_DM_BACKEND=0`
for persistence on a card or `1` for RAM. RAM missions do not survive a reboot.

## Build validation

Both targets built successfully with GNU Make and the existing toolchain on
2026-10-07, against checkout `b18a9dcc2f` with these board changes:

| Image | Binary size | Flash budget |
| --- | --- | --- |
| Bootloader | 39,460 bytes | 131,072 bytes |
| Firmware | 1,786,540 bytes | 1,835,008 bytes |

The firmware has 48,468 bytes of flash headroom. This configuration includes
DroneCAN and SD logging; Micro XRCE-DDS and several optional EKF2 features are
disabled to fit the flash budget.

Checks passed for firmware-package payload integrity, matching compiled board
identity, vector/reset addresses, application/parameter separation, required
driver symbols and the bundled bootloader. All 12 actuator mappings agree with
the independent INAV definition, all 26 serial/SPI/I2C/CAN GPIO aliases resolve
to their documented pins, and generated actuator metadata exposes S11/S12 as
PWM-only. Both targets appear in PX4's CI target discovery. The compiled USB
names pass QGC 4.4.3 and 5.0.8 detection checks; the current fallback table also
matches. Scoped PX4 C/C++ formatting checks and startup-script syntax checks pass.

These are build and static checks. USB enumeration, sensor operation, SD/CAN
traffic and actuator waveforms have not been tested on a BLITZ Wing board.

## Bench validation

Keep propellers removed. Check USB handoff, QGC connection, `ver all`,
`icm42688p status`, `dps310 -I -b 2 status`, `listener sensor_accel 4`, `mount`,
`df`, `logger status`, parameter persistence and mission persistence with a card.
Use `uavcan status` after enabling a correctly terminated CAN bus. Verify SD
logging/write latency and recovery without a card. Verify battery measurements,
GPS, RC failsafe, analog/digital airspeed, OSD and all actuator assignments.
Use a scope to check PWM/DShot signals and confirm servo BEC voltage before
connecting servos. Confirm level Z acceleration near -9.81 m/s² and expected
roll/pitch/yaw signs. Hardware/flight test logs are required before release.
