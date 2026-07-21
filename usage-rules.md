<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# BB.Sensor.BMI323 Usage Rules

`bb_sensor_bmi323` provides `BB.Sensor.BMI323`, a `BB.Sensor` driver for the
Bosch BMI323 6-DoF IMU (3-axis accelerometer + 3-axis gyroscope) over I2C, for
[Beam Bots](https://hexdocs.pm/bb). For BB framework basics, see `bb`'s rules
(`mix usage_rules.sync <file> bb:all`); this file covers only what's specific to
this sensor.

## Core principles

1. **It's a sensor you wire into the topology, not a module you call.** Attach
   it to the link the IMU is mounted on with `sensor :name, {BB.Sensor.BMI323,
   opts}`. BB supervises the process and injects `:bb => %{robot, path}` — you
   never start it yourself.
2. **No magnetometer means no orientation.** Every published `Imu` carries
   `BB.Math.Quaternion.identity/0` for `orientation`. To get a real orientation,
   pair it with an estimator from
   [`bb_estimator_ahrs`](https://hexdocs.pm/bb_estimator_ahrs).
3. **Pick the mode by output data rate.** `:polling` for ODR ≤ 200 Hz;
   `:interrupt` (FIFO watermark on an INT1 GPIO) for higher rates, up to the
   chip's 6.4 kHz.

## Wiring it in

```elixir
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
```

Interrupt mode needs INT1 wired to a host GPIO — set `mode: :interrupt` and
`int1_pin:`, and samples arrive in bursts of `watermark_frames`.

`mix igniter.install bb_sensor_bmi323` wires a `[:config, :bmi323]` param group
(`bus`, `address`, `int1_pin`) into the robot; reference those in the sensor
opts with `bus: param([:config, :bmi323, :bus])` so they're runtime-adjustable.

## The published message

`BB.Message.Sensor.Imu`, published on `[:sensor | path]`:

- `angular_velocity` — `BB.Math.Vec3`, rad/s (gyroscope).
- `linear_acceleration` — `BB.Math.Vec3`, m/s² (accelerometer).
- `orientation` — always `BB.Math.Quaternion.identity/0` (see above).

Subscribe by path and keep the whole `%BB.Message{}` — don't unwrap early:

```elixir
BB.subscribe(MyRobot.Robot, [:sensor, :base, :imu])

def handle_info({:bb, _path, %BB.Message{payload: %BB.Message.Sensor.Imu{} = imu}}, state) do
  # imu.angular_velocity, imu.linear_acceleration
  {:noreply, state}
end
```

## Key options

| Option | Default | Notes |
|---|---|---|
| `bus` | _required_ | I2C bus name, e.g. `"i2c-1"` |
| `address` | `0x68` | `0x68` (SDO→GND) or `0x69` (SDO→VDDIO) |
| `mode` | `:polling` | `:polling` or `:interrupt` |
| `int1_pin` | `nil` | GPIO for INT1 — **required** when `mode: :interrupt` |
| `publish_rate` | `~u(100 hertz)` | Polling only; ignored in interrupt mode |
| `watermark_frames` | `8` | FIFO frames per interrupt; interrupt only |
| `accelerometer_range` / `_odr` / `_mode` | `8` / `200` / `:normal` | g, Hz, power mode |
| `gyroscope_range` / `_odr` / `_mode` | `2000` / `200` / `:normal` | °/s, Hz, power mode |

Changing `publish_rate` or the `accelerometer_*`/`gyroscope_*` options is applied
live via `handle_options/2`; changing `mode`, `bus`, `address`, `int1_pin`, or
`watermark_frames` returns `{:stop, :reconfigure}` and the supervisor restarts
the sensor.

## Anti-patterns

- **Don't trust `orientation`.** It is always the identity quaternion. Attach a
  `bb_estimator_ahrs` estimator to fuse a real orientation and republish.
- **Don't assume one sample per message in interrupt mode.** Frames arrive in
  bursts of `watermark_frames`; design consumers to handle a batch (an AHRS
  estimator integrates each sample with its own `dt`, so it copes naturally).
- **Don't set `mode: :interrupt` without `int1_pin`** (crashes with
  `:int1_pin_required_for_interrupt_mode`), and don't leave an axis on
  `*_mode: :disabled` expecting valid data — that powers it down.

## Further reading

- [bb_sensor_bmi323 docs](https://hexdocs.pm/bb_sensor_bmi323) — full
  `BB.Sensor.BMI323` module docs, coordinate frame, and troubleshooting.
- `bb`'s PubSub/sensors rules (`bb:pubsub-and-sensors`) and
  [Sensors and PubSub](https://hexdocs.pm/bb/03-sensors-and-pubsub.html).
