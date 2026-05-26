# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sensor.BMI323 do
  @moduledoc """
  A BB sensor that publishes `BB.Message.Sensor.Imu` messages from a Bosch
  BMI323 6-DoF inertial measurement unit (3-axis accelerometer + 3-axis
  gyroscope) over I2C.

  The BMI323 has no magnetometer, so this sensor cannot determine
  orientation on its own — every published `Imu` carries
  `BB.Math.Quaternion.identity/0` for the `orientation` field. **You
  almost always want to pair this sensor with an orientation estimator**
  such as the ones in
  [`bb_estimator_ahrs`](https://hex.pm/packages/bb_estimator_ahrs)
  (Madgwick, Mahony, Complementary). See
  [Pairing with an AHRS estimator](#module-pairing-with-an-ahrs-estimator).

  ## Modes

  Pick based on output data rate:

  - `:polling` — periodically calls `BMI323.read_imu/1` at
    `publish_rate`. Low-overhead, low-jitter at modest rates. Use when
    ODR ≤ 200 Hz. Above ~200 Hz the BEAM scheduler starts to lose
    samples between polls.
  - `:interrupt` — runs `BMI323.Sampler` which buffers samples in the
    chip's on-chip 2 KB FIFO and fires the host's INT1 GPIO when a
    configurable watermark is reached. Reliable up to the chip's
    6.4 kHz ODR. Requires INT1 wired to a host GPIO.

  In interrupt mode, samples arrive in bursts of `watermark_frames` at
  once. With ODR 800 Hz and watermark 8, you'll see batches every 10 ms.
  Downstream consumers should be designed to handle a burst (an estimator
  like the AHRS filters integrates each sample with its own dt, so it
  copes naturally).

  ## Coordinate frame

  Axes are the chip's own +X / +Y / +Z (see the BMI323 datasheet
  §3.2 for the silkscreen orientation). The BB topology entity this
  sensor attaches to *is* its coordinate frame — orient the IMU on the
  link as appropriate and apply a static transform in your downstream
  consumer if the chip mounting axes don't match the link axes.

  ## Example DSL Usage

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

  Or in interrupt mode, with INT1 wired to GPIO 17:

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

  ## Pairing with an AHRS estimator

  The BMI323 produces raw acceleration + angular-velocity samples; turning
  those into an orientation needs sensor fusion. Attach an estimator from
  `bb_estimator_ahrs`:

      sensor :imu, {BB.Sensor.BMI323, bus: "i2c-1", ...} do
        estimator :orientation, {BB.Estimator.Ahrs.Madgwick, beta: 0.1}
      end

  The estimator subscribes to this sensor's `Imu` messages, replaces the
  identity quaternion with a fused orientation, and republishes. See
  `BB.Estimator.Ahrs.Madgwick`, `BB.Estimator.Ahrs.Mahony`, and
  `BB.Estimator.Ahrs.Complementary` for the algorithm choices.

  ## Options

  - `bus` — I2C bus name (e.g. `"i2c-1"`) — required.
  - `address` — I2C address (`0x68` or `0x69`, default `0x68`).
  - `mode` — `:polling` or `:interrupt` (default `:polling`).
  - `accelerometer_range` — `2 | 4 | 8 | 16` g (default `8`).
  - `accelerometer_odr` — output data rate in Hz (default `200`).
  - `accelerometer_mode` — `:disabled | :low_power | :normal |
    :high_performance` (default `:normal`). Setting either axis to
    `:disabled` powers it down — the IMU will publish constant /
    invalid values for that axis until the mode is changed back.
  - `gyroscope_range` — `125 | 250 | 500 | 1000 | 2000` °/s (default
    `2000`).
  - `gyroscope_odr` — output data rate in Hz (default `200`).
  - `gyroscope_mode` — as for accelerometer (default `:normal`).
  - `publish_rate` — polling rate (default `~u(100 hertz)`). Ignored in
    interrupt mode.
  - `int1_pin` — GPIO pin number wired to BMI323's INT1. Required when
    `mode: :interrupt`.
  - `watermark_frames` — FIFO frames per interrupt (default `8`).
    Interrupt mode only.

  ## Published Messages

  `BB.Message.Sensor.Imu` published to `[:sensor | path]` where `path` is
  the sensor's position in the topology. Fields:

  - `angular_velocity` — gyroscope reading as `BB.Math.Vec3` in rad/s.
  - `linear_acceleration` — accelerometer reading as `BB.Math.Vec3` in
    m/s².
  - `orientation` — `BB.Math.Quaternion.identity/0` (see above).

  ## Runtime parameter changes

  Options handled live (no restart):

  - `publish_rate` (polling mode) — interval is recomputed.
  - `accelerometer_range` / `accelerometer_odr` / `accelerometer_mode`
    — `BMI323.configure_accelerometer/2` is re-issued.
  - `gyroscope_range` / `gyroscope_odr` / `gyroscope_mode` —
    `BMI323.configure_gyroscope/2` is re-issued.

  Options that trigger `{:stop, :reconfigure}` (supervisor restarts the
  sensor with new params):

  - `mode`, `bus`, `address`, `int1_pin`, `watermark_frames`.

  ## Error handling

  A single failed read or reconfiguration is logged at warning level and
  does not crash the process — the polling loop or interrupt handler
  continues. Persistent failures will manifest as silence on the topic
  rather than a crash.

  ## Troubleshooting

  - `{:stop, {:chip_id_mismatch, got: id, expected: 0x43}}` — the device
    at the configured I2C address is not a BMI323. Check `address`
    (`0x68` when SDO is tied to GND, `0x69` when tied to VDDIO), the
    physical bus, and that the chip is actually powered.
  - `{:stop, :no_such_bus}` from `Wafer.Driver.Circuits.I2C.acquire/1`
    — the `bus` string doesn't match any `/dev/i2c-*` device. On Linux
    list available buses with `i2cdetect -l`.
  - `{:stop, :int1_pin_required_for_interrupt_mode}` — `mode:
    :interrupt` was set without an `int1_pin`.
  - GPIO acquire failure in interrupt mode — the pin may already be
    exported, owned by another process, or not exist on this board.
  - Constant / silently-wrong samples after changing `*_mode` — check
    you didn't leave an axis on `:disabled`; that powers it down.
  """

  use BB.Sensor

  import BB.Unit
  import BB.Unit.Option

  alias BB.Math.Quaternion
  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Sensor.Imu
  alias BB.Robot.Units
  alias Localize.Unit
  alias Wafer.Driver.Circuits.GPIO, as: CircuitsGPIO
  alias Wafer.Driver.Circuits.I2C, as: CircuitsI2C

  require Logger

  @modes [:polling, :interrupt]
  @accel_ranges [2, 4, 8, 16]
  @gyro_ranges [125, 250, 500, 1000, 2000]
  @power_modes [:disabled, :low_power, :normal, :high_performance]
  @valid_odrs [
    0.78125,
    1.5625,
    3.125,
    6.25,
    12.5,
    25,
    50,
    100,
    200,
    400,
    800,
    1600,
    3200,
    6400
  ]

  @impl BB.Sensor
  def options_schema do
    Spark.Options.new!(
      bus: [
        type: :string,
        required: true,
        doc: "I2C bus name (e.g. \"i2c-1\")"
      ],
      address: [
        type: :integer,
        default: 0x68,
        doc: "I2C address of the BMI323 (`0x68` or `0x69`)"
      ],
      mode: [
        type: {:in, @modes},
        default: :polling,
        doc: "Operating mode: `:polling` or `:interrupt`"
      ],
      accelerometer_range: [
        type: {:in, @accel_ranges},
        default: 8,
        doc: "Accelerometer full-scale range in g"
      ],
      accelerometer_odr: [
        type: {:in, @valid_odrs},
        default: 200,
        doc: "Accelerometer output data rate in Hz"
      ],
      accelerometer_mode: [
        type: {:in, @power_modes},
        default: :normal,
        doc: "Accelerometer power mode"
      ],
      gyroscope_range: [
        type: {:in, @gyro_ranges},
        default: 2000,
        doc: "Gyroscope full-scale range in °/s"
      ],
      gyroscope_odr: [
        type: {:in, @valid_odrs},
        default: 200,
        doc: "Gyroscope output data rate in Hz"
      ],
      gyroscope_mode: [
        type: {:in, @power_modes},
        default: :normal,
        doc: "Gyroscope power mode"
      ],
      publish_rate: [
        type: unit_type(compatible: :hertz),
        default: ~u(100 hertz),
        doc: "Polling rate (polling mode only)"
      ],
      int1_pin: [
        type: {:or, [:non_neg_integer, nil]},
        default: nil,
        doc: "GPIO pin number wired to BMI323 INT1 (required for `:interrupt` mode)"
      ],
      watermark_frames: [
        type: :pos_integer,
        default: 8,
        doc: "FIFO frames per interrupt (interrupt mode only)"
      ]
    )
  end

  @impl BB.Sensor
  def init(opts) do
    opts = Map.new(opts)

    with :ok <- validate_mode(opts),
         {:ok, bmi} <- acquire_chip(opts),
         {:ok, bmi} <- configure_accelerometer(bmi, opts),
         {:ok, bmi} <- configure_gyroscope(bmi, opts),
         {:ok, state} <- start_mode(bmi, opts) do
      {:ok, state}
    else
      {:error, reason} -> {:stop, reason}
    end
  end

  @impl BB.Sensor
  def handle_info(:tick, %{mode: :polling} = state) do
    case BMI323.read_imu(state.bmi) do
      {:ok, sample} ->
        publish_sample(state, sample)

      {:error, reason} ->
        Logger.warning("BMI323 read failed at #{inspect(state.bb.path)}: #{inspect(reason)}")
    end

    schedule_tick(state.publish_interval_ms)
    {:noreply, state}
  end

  def handle_info({BMI323.Sampler, _pid, frames}, %{mode: :interrupt} = state) do
    Enum.each(frames, &publish_frame(state, &1))
    {:noreply, state}
  end

  def handle_info(_other, state), do: {:noreply, state}

  @structural_keys [:mode, :bus, :address, :int1_pin, :watermark_frames]

  @impl BB.Sensor
  def handle_options(new_opts, state) do
    new_opts = Map.new(new_opts)

    if structural_change?(new_opts, state.opts) do
      {:stop, :reconfigure}
    else
      apply_live_changes(new_opts, state)
    end
  end

  defp structural_change?(new_opts, old_opts) do
    Enum.any?(@structural_keys, fn key ->
      Map.get(new_opts, key) != Map.get(old_opts, key)
    end)
  end

  defp apply_live_changes(new_opts, state) do
    with {:ok, bmi} <- maybe_reconfigure_accel(state.bmi, new_opts, state.opts),
         {:ok, bmi} <- maybe_reconfigure_gyro(bmi, new_opts, state.opts) do
      publish_interval_ms = hertz_to_ms(new_opts.publish_rate)

      {:ok, %{state | bmi: bmi, publish_interval_ms: publish_interval_ms, opts: new_opts}}
    else
      {:error, reason} -> {:stop, reason}
    end
  end

  @accel_keys [:accelerometer_range, :accelerometer_odr, :accelerometer_mode]
  @gyro_keys [:gyroscope_range, :gyroscope_odr, :gyroscope_mode]

  defp maybe_reconfigure_accel(bmi, new_opts, old_opts) do
    if Enum.any?(@accel_keys, &(Map.get(new_opts, &1) != Map.get(old_opts, &1))) do
      configure_accelerometer(bmi, new_opts)
    else
      {:ok, bmi}
    end
  end

  defp maybe_reconfigure_gyro(bmi, new_opts, old_opts) do
    if Enum.any?(@gyro_keys, &(Map.get(new_opts, &1) != Map.get(old_opts, &1))) do
      configure_gyroscope(bmi, new_opts)
    else
      {:ok, bmi}
    end
  end

  defp validate_mode(%{mode: :interrupt, int1_pin: nil}),
    do: {:error, :int1_pin_required_for_interrupt_mode}

  defp validate_mode(_opts), do: :ok

  defp acquire_chip(opts) do
    with {:ok, conn} <-
           CircuitsI2C.acquire(bus_name: opts.bus, address: opts.address) do
      BMI323.acquire(conn: conn, soft_reset: true)
    end
  end

  defp configure_accelerometer(bmi, opts) do
    BMI323.configure_accelerometer(bmi,
      mode: opts.accelerometer_mode,
      odr: opts.accelerometer_odr,
      range: opts.accelerometer_range
    )
  end

  defp configure_gyroscope(bmi, opts) do
    BMI323.configure_gyroscope(bmi,
      mode: opts.gyroscope_mode,
      odr: opts.gyroscope_odr,
      range: opts.gyroscope_range
    )
  end

  defp start_mode(bmi, %{mode: :polling} = opts) do
    publish_interval_ms = hertz_to_ms(opts.publish_rate)

    state = %{
      bb: opts.bb,
      bmi: bmi,
      mode: :polling,
      opts: opts,
      publish_interval_ms: publish_interval_ms
    }

    schedule_tick(publish_interval_ms)
    {:ok, state}
  end

  defp start_mode(bmi, %{mode: :interrupt} = opts) do
    with {:ok, int1} <- CircuitsGPIO.acquire(pin: opts.int1_pin, direction: :in),
         {:ok, sampler} <-
           BMI323.Sampler.start_link(
             bmi: bmi,
             int1: int1,
             subscriber: self(),
             sources: [:accelerometer, :gyroscope],
             watermark_frames: opts.watermark_frames
           ) do
      state = %{
        bb: opts.bb,
        bmi: bmi,
        mode: :interrupt,
        opts: opts,
        sampler: sampler
      }

      {:ok, state}
    end
  end

  defp publish_sample(state, %{accelerometer: accel, gyroscope: gyro}) do
    publish_imu(state, accel, gyro)
  end

  defp publish_frame(state, %{accelerometer: accel, gyroscope: gyro}) do
    publish_imu(state, accel, gyro)
  end

  defp publish_frame(_state, _frame), do: :ok

  defp publish_imu(state, accel, gyro) do
    frame_id = List.last(state.bb.path)

    {:ok, message} =
      Message.new(Imu, frame_id,
        orientation: Quaternion.identity(),
        angular_velocity: Vec3.new(gyro.x, gyro.y, gyro.z),
        linear_acceleration: Vec3.new(accel.x, accel.y, accel.z)
      )

    BB.publish(state.bb.robot, [:sensor | state.bb.path], message)
  end

  defp hertz_to_ms(rate) do
    rate
    |> Unit.convert!("hertz")
    |> Units.extract_float()
    |> then(&round(1000 / &1))
  end

  defp schedule_tick(ms) do
    Process.send_after(self(), :tick, ms)
  end
end
