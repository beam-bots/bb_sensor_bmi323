<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# bb_sensor_bmi323

[Beam Bots](https://github.com/beam-bots/bb) integration for the Bosch
[BMI323](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi323/)
6-DoF inertial measurement unit (accelerometer + gyroscope) over I2C.

Operates the chip in either polling mode (periodic register reads) or
interrupt mode (FIFO watermark on INT1) and publishes
`BB.Message.Sensor.Imu` messages with angular velocity (rad/s) and linear
acceleration (m/s²). The BMI323 has no magnetometer, so `orientation` is
published as the identity quaternion — pair this sensor with
`bb_estimator_ahrs` to compute a real orientation.

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

Wire the chip's INT1 pin to a GPIO and let the on-chip FIFO buffer samples:

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

Subscribe to readings:

```elixir
BB.subscribe(MyRobot, [:sensor, :base, :imu])
```

See `BB.Sensor.BMI323` for full options.

## Options

| Option                 | Default       | Description                                                |
| ---------------------- | ------------- | ---------------------------------------------------------- |
| `bus`                  | _required_    | I2C bus name (e.g. `"i2c-1"`)                              |
| `address`              | `0x68`        | I2C address (`0x68` or `0x69`)                             |
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
