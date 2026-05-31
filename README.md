<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

<img src="https://github.com/beam-bots/bb/blob/main/logos/beam_bots_logo.png?raw=true" alt="Beam Bots Logo" width="250" />

# bb_sensor_bmi323

[![CI](https://github.com/beam-bots/bb_sensor_bmi323/actions/workflows/ci.yml/badge.svg)](https://github.com/beam-bots/bb_sensor_bmi323/actions/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](https://opensource.org/licenses/Apache-2.0)
[![Hex version badge](https://img.shields.io/hexpm/v/bb_sensor_bmi323.svg)](https://hex.pm/packages/bb_sensor_bmi323)
[![Hexdocs badge](https://img.shields.io/badge/docs-hexdocs-purple)](https://hexdocs.pm/bb_sensor_bmi323)
[![REUSE status](https://api.reuse.software/badge/github.com/beam-bots/bb_sensor_bmi323)](https://api.reuse.software/info/github.com/beam-bots/bb_sensor_bmi323)
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/beam-bots/bb_sensor_bmi323)

[Beam Bots](https://github.com/beam-bots/bb) integration for the Bosch
[BMI323](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi323/)
6-DoF inertial measurement unit (accelerometer + gyroscope) over I2C.

Operates the chip in either *polling* mode (periodic register reads) or
*interrupt* mode (FIFO watermark on INT1) and publishes
`BB.Message.Sensor.Imu` messages with angular velocity (rad/s) and linear
acceleration (m/s²).

The BMI323 has no magnetometer, so `orientation` is published as the
identity quaternion. **You almost always want to pair this sensor with an
orientation estimator** such as
[`bb_estimator_ahrs`](https://hex.pm/packages/bb_estimator_ahrs).

## Choosing a mode

| ODR ≤ 200 Hz                          | ODR > 200 Hz                          |
| ------------------------------------- | ------------------------------------- |
| `mode: :polling` — simple, no GPIO    | `mode: :interrupt` — needs INT1 wired |
| Low jitter, low overhead              | Reliable up to the chip's 6.4 kHz ODR |
| No FIFO, samples read one at a time   | FIFO-buffered, samples arrive in bursts of `watermark_frames` |

In interrupt mode, a `watermark_frames: 8` setting at ODR 800 Hz means
batches every 10 ms. Downstream consumers (AHRS estimators, kinematics
filters) typically cope with bursts naturally since each sample's `dt` is
read from its own monotonic timestamp.

## Usage

### Polling mode

```elixir
defmodule MyRobot do
  use BB

  topology do
    link :base do
      sensor :imu, {BB.Sensor.BMI323,
        bus: "i2c-1",
        address: 0x68,
        mode: :polling,
        accelerometer_range: 8,
        accelerometer_odr: 200,
        gyroscope_range: 2000,
        gyroscope_odr: 200,
        publish_rate: ~u(100 hertz)
      }
    end
  end
end
```

### Interrupt mode

Wire the chip's INT1 pin to a GPIO and let the on-chip FIFO buffer
samples:

```elixir
topology do
  link :base do
    sensor :imu, {BB.Sensor.BMI323,
      bus: "i2c-1",
      address: 0x68,
      mode: :interrupt,
      int1_pin: 17,
      accelerometer_range: 8,
      accelerometer_odr: 800,
      gyroscope_range: 2000,
      gyroscope_odr: 800,
      watermark_frames: 8
    }
  end
end
```

### Pairing with an AHRS estimator

The whole point of the identity `orientation` field is that an estimator
fills it in:

```elixir
topology do
  link :base do
    sensor :imu, {BB.Sensor.BMI323, bus: "i2c-1", ...} do
      estimator :orientation, {BB.Estimator.Ahrs.Madgwick, beta: 0.1}
    end
  end
end
```

The estimator subscribes to the sensor's `Imu` messages, replaces the
identity quaternion with a fused orientation, and republishes. See
`bb_estimator_ahrs` for the three available algorithms (Madgwick, Mahony,
Complementary) and their tuning options.

Subscribe to the final stream:

```elixir
BB.subscribe(MyRobot, [:sensor, :base, :imu])
```

## Coordinate frame

Axes are the chip's own +X / +Y / +Z silkscreen (see the BMI323 datasheet
§3.2). The BB topology entity this sensor attaches to *is* its coordinate
frame — orient the IMU on the link as appropriate and apply a static
transform downstream if the chip mounting doesn't match link axes.

## Options

| Option                 | Default       | Description                                                |
| ---------------------- | ------------- | ---------------------------------------------------------- |
| `bus`                  | _required_    | I2C bus name (e.g. `"i2c-1"`)                              |
| `address`              | `0x68`        | I2C address (`0x68` SDO→GND, `0x69` SDO→VDDIO)             |
| `mode`                 | `:polling`    | `:polling` or `:interrupt`                                 |
| `accelerometer_range`  | `8`           | g (`2`, `4`, `8`, `16`)                                    |
| `accelerometer_odr`    | `200`         | Hz (`12.5`..`6400`)                                        |
| `accelerometer_mode`   | `:normal`     | `:normal`, `:low_power`, `:high_performance`, `:disabled`  |
| `gyroscope_range`      | `2000`        | °/s (`125`, `250`, `500`, `1000`, `2000`)                  |
| `gyroscope_odr`        | `200`         | Hz                                                         |
| `gyroscope_mode`       | `:normal`     | as for accelerometer                                       |
| `publish_rate`         | `~u(100 hertz)` | Polling rate (polling mode only)                         |
| `int1_pin`             | _required if `mode: :interrupt`_ | GPIO pin number wired to BMI323 INT1   |
| `watermark_frames`     | `8`           | FIFO frames per interrupt (interrupt mode only)            |

Setting `accelerometer_mode` or `gyroscope_mode` to `:disabled` powers
that axis down — the IMU will publish constant / invalid values for it
until the mode is changed back.

## Runtime parameter changes

Live (no restart):

- `publish_rate` (polling mode) — interval is recomputed.
- `accelerometer_*` / `gyroscope_*` — chip is reconfigured.

Triggers `{:stop, :reconfigure}` (supervisor restarts with new params):

- `mode`, `bus`, `address`, `int1_pin`, `watermark_frames`.

## Troubleshooting

- **`{:chip_id_mismatch, got: id, expected: 0x43}`** — the device at the
  configured I2C address isn't a BMI323. Check `address` (`0x68` SDO→GND,
  `0x69` SDO→VDDIO), the bus, and that the chip is powered.
- **`:no_such_bus`** — the `bus` string doesn't match any `/dev/i2c-*`
  device. On Linux: `i2cdetect -l`.
- **`:int1_pin_required_for_interrupt_mode`** — `mode: :interrupt` was
  set without an `int1_pin`.
- **GPIO acquire failure** in interrupt mode — pin may already be
  exported, owned by another process, or not exist on this board.
- **Constant / silently-wrong samples after changing `*_mode`** — make
  sure you didn't leave an axis on `:disabled`.

See `BB.Sensor.BMI323` for full module documentation.
