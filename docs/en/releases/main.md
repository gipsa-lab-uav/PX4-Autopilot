# PX4-Autopilot Main Release Notes

<Badge type="danger" text="Alpha" />

<script setup>
import { useData } from 'vitepress'
const { site } = useData();
</script>

<div v-if="site.title !== 'PX4 Guide (main)'">
  <div class="custom-block danger">
    <p class="custom-block-title">This page is on a release branch, and hence probably out of date. <a href="https://docs.px4.io/main/en/releases/main">See the latest version</a>.</p>
  </div>
  <Redirect to="https://docs.px4.io/main/en/releases/main" />
</div>

This contains changes to the PX4 `main` branch that are not included in the next release ([PX4 v1.18](../releases/1.18.md)).

::: warning
PX4 v1.18 is in beta testing.
Update these notes with features that are going to be in `main` (PX4 v1.19 or later) but not the PX4 v1.18 release.
:::

## Read Before Upgrading

Please continue reading for [upgrade instructions](#upgrade-guide).

## Major Changes

- TBD

## Upgrade Guide

## Other changes

- Fast mission Return modes ([RTL_TYPE](../advanced_config/parameter_reference.md#RTL_TYPE) = 2 and 4) now skip `DO_JUMP` commands (loops) while following the mission path. ([PX4-Autopilot#26993: fix(navigator): goToNextPositionItem skip loops when required](https://github.com/PX4/PX4-Autopilot/pull/26993))

### Hardware Support

- TBD

### Common

- TBD

### Control

- TBD

### Safety

- TBD

### Safety

- Rotary-wing vehicles now support uncommanded altitude loss detection: if the vehicle descends more than [FD_ALT_LOSS](../advanced_config/parameter_reference.md#FD_ALT_LOSS) meters below its setpoint in altitude-controlled flight, flight termination (and parachute deployment) is triggered. See [Altitude Loss Trigger](../config/safety.md#altitude-loss-trigger). ([PX4-Autopilot#26837](https://github.com/PX4/PX4-Autopilot/pull/26837))
- [Parachute health failsafe](../peripherals/parachute.md): extended [COM_PARACHUTE](../advanced_config/parameter_reference.md#COM_PARACHUTE) from a boolean into a configurable in-flight failsafe action (Return or Land) parameter. The previously enabled value `COM_PARACHUTE=1` causes a warning like before. ([PX4-Autopilot#26918](https://github.com/PX4/PX4-Autopilot/pull/26918))
- [GNSS check failsafe](../config/safety.md#gnss-check-failsafe): new failsafe that monitors the number of usable GNSS receivers with a 3D fix and their position consistency. The required number of receivers is set via [SYS_HAS_NUM_GNSS](../advanced_config/parameter_reference.md#SYS_HAS_NUM_GNSS) and the failsafe action via [COM_GNSSLOSS_ACT](../advanced_config/parameter_reference.md#COM_GNSSLOSS_ACT). ([PX4-Autopilot#26863](https://github.com/PX4/PX4-Autopilot/pull/26863))

### Estimation

- Added [EKF2_POS_LOCK](../advanced_config/parameter_reference.md#EKF2_POS_LOCK) to force constant position fusion while landed, useful for vehicles relying on dead-reckoning sensors (airspeed, optical flow) that provide no aiding on the ground.

### Sensors

- TBD

### Simulation

- SIH: Add option to set wind velocity ([PX4-Autopilot#26467](https://github.com/PX4-Autopilot/pull/26467))

### Debug & Logging

- TBD

### Ethernet

- TBD

### uXRCE-DDS / Zenoh / ROS2

- TBD

### MAVLink

- Removed support for deprecated request commands `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES`, `MAV_CMD_REQUEST_PROTOCOL_VERSION`, `MAV_CMD_GET_HOME_POSITION`, `MAV_CMD_REQUEST_FLIGHT_INFORMATION`, `MAV_CMD_REQUEST_STORAGE_INFORMATION` (Replaced by `MAV_CMD_REQUEST_MESSAGE`).
  ([PX4-Autopilot#27251: fix(mavlink): Remove deprecated MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES](https://github.com/PX4/PX4-Autopilot/pull/27251), [PX4-Autopilot#27252: fix(mavlink): Remove legacy mavlink message requestors#27252](https://github.com/PX4/PX4-Autopilot/pull/27252))

### RC

- Parse ELRS Status and Link Statistics TX messages in the CRSF parser.

### RC

- TBD

### Multi-Rotor

- TBD

### VTOL

- TBD

### Fixed-wing

- TBD

### Rover

- TBD

### ROS 2

- TBD
