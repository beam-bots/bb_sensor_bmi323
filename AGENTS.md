# AGENTS.md

This file provides guidance to AI coding agents when working with code in this repository.

## Project Overview

`bb_sensor_bmi323` is a Beam Bots integration library for the Bosch BMI323
6-DoF IMU (3-axis accelerometer + 3-axis gyroscope) over I2C. A single
`BB.Sensor.BMI323` sensor module operates the chip in either *polling* mode
(periodic `BMI323.read_imu/1`) or *interrupt* mode (FIFO watermark on INT1
GPIO via `BMI323.Sampler`) and publishes `BB.Message.Sensor.Imu` messages.

## Build and Test Commands

```bash
mix check --no-retry    # Run all checks (compile, test, format, credo, dialyzer, reuse)
mix test                # Run tests
mix test path/to/test.exs:42  # Single test at line
mix format
mix credo --strict
```

Prefer `mix check --no-retry` over running individual tools.

## Architecture

A single module, `BB.Sensor.BMI323` (`lib/bb/sensor/bmi323.ex`), implementing
the `BB.Sensor` behaviour:

- `init/1` opens the I2C bus via `Wafer.Driver.Circuits.I2C.acquire/1`,
  calls `BMI323.acquire/1` (with soft reset), configures the accelerometer
  and gyroscope, then either schedules a poll tick (polling mode) or starts
  a linked `BMI323.Sampler` GenServer wired to INT1 (interrupt mode).
- `handle_info(:tick, state)` (polling mode) reads one IMU sample via
  `BMI323.read_imu/1`, publishes a `BB.Message.Sensor.Imu` on `[:sensor |
  path]`, and reschedules.
- `handle_info({BMI323.Sampler, _pid, frames}, state)` (interrupt mode)
  publishes one `BB.Message.Sensor.Imu` per frame.
- `handle_options/2` recomputes the publish interval when `publish_rate` is
  bound to a runtime parameter (polling mode only). Other options require
  a restart to take effect.

The BMI323 has no magnetometer, so published `Imu` messages carry an
identity quaternion for `orientation`. Pair this sensor with
`bb_estimator_ahrs` (Madgwick / Mahony / Complementary) to compute a real
orientation downstream.

## Units

SI throughout: m/s² for linear acceleration, rad/s for angular velocity.
The underlying `BMI323` library already returns scaled values; the sensor
wraps them in `BB.Math.Vec3` structs for the `Imu` message.

## Testing

Tests use Mimic to mock `BB`, `BMI323`, `BMI323.Sampler`,
`Wafer.Driver.Circuits.I2C`, and `Wafer.Driver.Circuits.GPIO`. Test support
modules live in `test/support/`.

## Dependencies

- `bb` — The Beam Bots robotics framework
- `bmi323` — Low-level BMI323 driver wrapping `Wafer.Conn`
- `wafer` — Hardware abstraction (we use `Wafer.Driver.Circuits.I2C` and
  `Wafer.Driver.Circuits.GPIO` directly)
- `circuits_i2c`, `circuits_gpio` — I2C and GPIO backends
