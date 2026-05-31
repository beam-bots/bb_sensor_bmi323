# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sensor.BMI323Test do
  use ExUnit.Case, async: true
  use Mimic

  import BB.Unit

  alias BB.Math.Quaternion
  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Sensor.Imu
  alias BB.Robot.Units
  alias BB.Sensor.BMI323, as: Sensor
  alias Localize.Unit

  @sensor_name :imu_test
  @sensor_path [:base, @sensor_name]

  defp default_bb_context do
    %{robot: TestRobot, path: @sensor_path, name: @sensor_name}
  end

  defp polling_opts(overrides \\ []) do
    [
      bb: default_bb_context(),
      bus: "i2c-1",
      address: 0x68,
      mode: :polling,
      accelerometer_range: 8,
      accelerometer_odr: 200,
      accelerometer_mode: :normal,
      gyroscope_range: 2000,
      gyroscope_odr: 200,
      gyroscope_mode: :normal,
      publish_rate: ~u(100 hertz),
      int1_pin: nil,
      watermark_frames: 8
    ]
    |> Keyword.merge(overrides)
  end

  defp interrupt_opts(overrides \\ []) do
    polling_opts(mode: :interrupt, int1_pin: 17) |> Keyword.merge(overrides)
  end

  defp fake_conn, do: :fake_wafer_conn
  defp fake_gpio, do: :fake_gpio_conn

  defp fake_bmi(extra \\ %{}) do
    Map.merge(
      %BMI323{conn: fake_conn(), accelerometer_range: 8, gyroscope_range: 2000},
      extra
    )
  end

  defp stub_acquire_success do
    stub(Wafer.Driver.Circuits.I2C, :acquire, fn _opts -> {:ok, fake_conn()} end)
    stub(BMI323, :acquire, fn _opts -> {:ok, fake_bmi()} end)
    stub(BMI323, :configure_accelerometer, fn bmi, _opts -> {:ok, bmi} end)
    stub(BMI323, :configure_gyroscope, fn bmi, _opts -> {:ok, bmi} end)
  end

  defp stub_interrupt_success do
    stub_acquire_success()
    stub(Wafer.Driver.Circuits.GPIO, :acquire, fn _opts -> {:ok, fake_gpio()} end)
    stub(BMI323.Sampler, :start_link, fn _opts -> {:ok, :fake_sampler_pid} end)
  end

  defp sample do
    %{
      accelerometer: %{x: 0.0, y: 0.0, z: 9.81},
      gyroscope: %{x: 0.1, y: 0.2, z: 0.3},
      temperature: 25.0
    }
  end

  defp drain_self_tick do
    receive do
      :tick -> :ok
    after
      50 -> :no_tick
    end
  end

  describe "init/1 (polling mode)" do
    test "acquires the chip, configures it, and returns state with tick scheduled" do
      stub_acquire_success()

      assert {:ok, state} = Sensor.init(polling_opts())
      assert state.mode == :polling
      assert state.bb == default_bb_context()
      assert state.publish_interval_ms == 10
      assert state.bmi == fake_bmi()
      assert state.opts.bus == "i2c-1"
      assert_receive :tick, 50
    end

    test "passes bus_name and address through to Wafer" do
      test_pid = self()

      expect(Wafer.Driver.Circuits.I2C, :acquire, fn opts ->
        send(test_pid, {:wafer_opts, opts})
        {:ok, fake_conn()}
      end)

      stub(BMI323, :acquire, fn _ -> {:ok, fake_bmi()} end)
      stub(BMI323, :configure_accelerometer, fn bmi, _ -> {:ok, bmi} end)
      stub(BMI323, :configure_gyroscope, fn bmi, _ -> {:ok, bmi} end)

      Sensor.init(polling_opts(bus: "i2c-3", address: 0x69))

      assert_receive {:wafer_opts, opts}
      assert opts[:bus_name] == "i2c-3"
      assert opts[:address] == 0x69
      drain_self_tick()
    end

    test "issues soft reset on BMI323.acquire" do
      stub(Wafer.Driver.Circuits.I2C, :acquire, fn _ -> {:ok, fake_conn()} end)
      stub(BMI323, :configure_accelerometer, fn bmi, _ -> {:ok, bmi} end)
      stub(BMI323, :configure_gyroscope, fn bmi, _ -> {:ok, bmi} end)
      test_pid = self()

      expect(BMI323, :acquire, fn opts ->
        send(test_pid, {:acquire_opts, opts})
        {:ok, fake_bmi()}
      end)

      Sensor.init(polling_opts())

      assert_receive {:acquire_opts, opts}
      assert opts[:soft_reset] == true
      drain_self_tick()
    end

    test "passes range / odr / mode through to configure_accelerometer" do
      stub(Wafer.Driver.Circuits.I2C, :acquire, fn _ -> {:ok, fake_conn()} end)
      stub(BMI323, :acquire, fn _ -> {:ok, fake_bmi()} end)
      stub(BMI323, :configure_gyroscope, fn bmi, _ -> {:ok, bmi} end)
      test_pid = self()

      expect(BMI323, :configure_accelerometer, fn bmi, opts ->
        send(test_pid, {:accel_opts, opts})
        {:ok, bmi}
      end)

      Sensor.init(
        polling_opts(
          accelerometer_range: 4,
          accelerometer_odr: 400,
          accelerometer_mode: :high_performance
        )
      )

      assert_receive {:accel_opts, opts}
      assert opts[:range] == 4
      assert opts[:odr] == 400
      assert opts[:mode] == :high_performance
      drain_self_tick()
    end

    test "passes range / odr / mode through to configure_gyroscope" do
      stub(Wafer.Driver.Circuits.I2C, :acquire, fn _ -> {:ok, fake_conn()} end)
      stub(BMI323, :acquire, fn _ -> {:ok, fake_bmi()} end)
      stub(BMI323, :configure_accelerometer, fn bmi, _ -> {:ok, bmi} end)
      test_pid = self()

      expect(BMI323, :configure_gyroscope, fn bmi, opts ->
        send(test_pid, {:gyro_opts, opts})
        {:ok, bmi}
      end)

      Sensor.init(
        polling_opts(
          gyroscope_range: 500,
          gyroscope_odr: 100,
          gyroscope_mode: :low_power
        )
      )

      assert_receive {:gyro_opts, opts}
      assert opts[:range] == 500
      assert opts[:odr] == 100
      assert opts[:mode] == :low_power
      drain_self_tick()
    end

    test "translates publish_rate to interval in milliseconds" do
      stub_acquire_success()

      assert {:ok, %{publish_interval_ms: 1000}} =
               Sensor.init(polling_opts(publish_rate: ~u(1 hertz)))

      drain_self_tick()

      assert {:ok, %{publish_interval_ms: 2}} =
               Sensor.init(polling_opts(publish_rate: ~u(500 hertz)))

      drain_self_tick()
    end

    test "stops on Wafer acquire failure" do
      expect(Wafer.Driver.Circuits.I2C, :acquire, fn _ -> {:error, :no_such_bus} end)

      assert {:stop, :no_such_bus} = Sensor.init(polling_opts())
    end

    test "stops on BMI323.acquire failure" do
      stub(Wafer.Driver.Circuits.I2C, :acquire, fn _ -> {:ok, fake_conn()} end)
      expect(BMI323, :acquire, fn _ -> {:error, :chip_id_mismatch} end)

      assert {:stop, :chip_id_mismatch} = Sensor.init(polling_opts())
    end

    test "stops on configure_accelerometer failure" do
      stub(Wafer.Driver.Circuits.I2C, :acquire, fn _ -> {:ok, fake_conn()} end)
      stub(BMI323, :acquire, fn _ -> {:ok, fake_bmi()} end)
      expect(BMI323, :configure_accelerometer, fn _, _ -> {:error, :bad_range} end)

      assert {:stop, :bad_range} = Sensor.init(polling_opts())
    end

    test "stops on configure_gyroscope failure" do
      stub(Wafer.Driver.Circuits.I2C, :acquire, fn _ -> {:ok, fake_conn()} end)
      stub(BMI323, :acquire, fn _ -> {:ok, fake_bmi()} end)
      stub(BMI323, :configure_accelerometer, fn bmi, _ -> {:ok, bmi} end)
      expect(BMI323, :configure_gyroscope, fn _, _ -> {:error, :bad_odr} end)

      assert {:stop, :bad_odr} = Sensor.init(polling_opts())
    end
  end

  describe "init/1 (interrupt mode)" do
    test "rejects missing int1_pin" do
      assert {:stop, :int1_pin_required_for_interrupt_mode} =
               Sensor.init(interrupt_opts(int1_pin: nil))
    end

    test "acquires the GPIO, starts the sampler, and returns state" do
      stub_interrupt_success()

      assert {:ok, state} = Sensor.init(interrupt_opts())
      assert state.mode == :interrupt
      assert state.bmi == fake_bmi()
      assert state.sampler == :fake_sampler_pid
    end

    test "passes pin to GPIO acquire" do
      stub_acquire_success()
      stub(BMI323.Sampler, :start_link, fn _ -> {:ok, :fake_sampler_pid} end)
      test_pid = self()

      expect(Wafer.Driver.Circuits.GPIO, :acquire, fn opts ->
        send(test_pid, {:gpio_opts, opts})
        {:ok, fake_gpio()}
      end)

      Sensor.init(interrupt_opts(int1_pin: 22))

      assert_receive {:gpio_opts, opts}
      assert opts[:pin] == 22
      assert opts[:direction] == :in
    end

    test "wires the sampler to receive interrupts on self()" do
      stub_acquire_success()
      stub(Wafer.Driver.Circuits.GPIO, :acquire, fn _ -> {:ok, fake_gpio()} end)
      test_pid = self()

      expect(BMI323.Sampler, :start_link, fn opts ->
        send(test_pid, {:sampler_opts, opts})
        {:ok, :fake_sampler_pid}
      end)

      Sensor.init(interrupt_opts(watermark_frames: 32))

      assert_receive {:sampler_opts, opts}
      assert opts[:int1] == fake_gpio()
      assert opts[:subscriber] == self()
      assert opts[:watermark_frames] == 32
      assert opts[:sources] == [:accelerometer, :gyroscope]
    end

    test "stops on GPIO acquire failure" do
      stub_acquire_success()
      expect(Wafer.Driver.Circuits.GPIO, :acquire, fn _ -> {:error, :no_such_pin} end)

      assert {:stop, :no_such_pin} = Sensor.init(interrupt_opts())
    end

    test "stops on sampler start_link failure" do
      stub_acquire_success()
      stub(Wafer.Driver.Circuits.GPIO, :acquire, fn _ -> {:ok, fake_gpio()} end)
      expect(BMI323.Sampler, :start_link, fn _ -> {:error, :no_chip} end)

      assert {:stop, :no_chip} = Sensor.init(interrupt_opts())
    end
  end

  describe "handle_info(:tick, state)" do
    setup do
      state = %{
        bb: default_bb_context(),
        bmi: fake_bmi(),
        mode: :polling,
        opts: Map.new(polling_opts()),
        publish_interval_ms: 1000
      }

      {:ok, state: state}
    end

    test "reads IMU and publishes an Imu message with identity orientation", %{state: state} do
      stub(BMI323, :read_imu, fn _ -> {:ok, sample()} end)
      test_pid = self()

      expect(BB, :publish, fn robot, path, %Message{payload: %Imu{} = payload} ->
        send(test_pid, {:published, robot, path, payload})
        :ok
      end)

      assert {:noreply, ^state} = Sensor.handle_info(:tick, state)

      assert_receive {:published, TestRobot, [:sensor, :base, @sensor_name], payload}
      assert payload.orientation == Quaternion.identity()
      assert payload.linear_acceleration == Vec3.new(0.0, 0.0, 9.81)
      assert payload.angular_velocity == Vec3.new(0.1, 0.2, 0.3)
      drain_self_tick()
    end

    test "frame_id is the sensor name", %{state: state} do
      stub(BMI323, :read_imu, fn _ -> {:ok, sample()} end)
      test_pid = self()

      expect(BB, :publish, fn _, _, %Message{} = msg ->
        send(test_pid, {:frame_id, msg.frame_id})
        :ok
      end)

      Sensor.handle_info(:tick, state)

      assert_receive {:frame_id, @sensor_name}
      drain_self_tick()
    end

    test "raises on read error so the supervisor sees the failure", %{state: state} do
      stub(BMI323, :read_imu, fn _ -> {:error, :no_device} end)
      reject(&BB.publish/3)

      assert_raise MatchError, fn -> Sensor.handle_info(:tick, state) end
      assert :no_tick = drain_self_tick()
    end

    test "reschedules a tick at publish_interval_ms", %{state: state} do
      stub(BMI323, :read_imu, fn _ -> {:ok, sample()} end)
      stub(BB, :publish, fn _, _, _ -> :ok end)

      state = %{state | publish_interval_ms: 10}
      Sensor.handle_info(:tick, state)

      assert_receive :tick, 50
    end
  end

  describe "handle_info({BMI323.Sampler, ...}, state)" do
    setup do
      state = %{
        bb: default_bb_context(),
        bmi: fake_bmi(),
        mode: :interrupt,
        opts: Map.new(interrupt_opts()),
        sampler: :fake_sampler_pid
      }

      {:ok, state: state}
    end

    test "publishes one Imu per frame in the batch", %{state: state} do
      frames = [
        %{accelerometer: %{x: 1.0, y: 0.0, z: 0.0}, gyroscope: %{x: 0.0, y: 0.0, z: 1.0}},
        %{accelerometer: %{x: 0.0, y: 1.0, z: 0.0}, gyroscope: %{x: 0.0, y: 1.0, z: 0.0}}
      ]

      test_pid = self()

      stub(BB, :publish, fn _, _, %Message{payload: %Imu{} = payload} ->
        send(test_pid, {:published, payload})
        :ok
      end)

      assert {:noreply, ^state} =
               Sensor.handle_info({BMI323.Sampler, :fake_sampler_pid, frames}, state)

      assert_receive {:published, p1}
      assert p1.linear_acceleration == Vec3.new(1.0, 0.0, 0.0)
      assert p1.angular_velocity == Vec3.new(0.0, 0.0, 1.0)

      assert_receive {:published, p2}
      assert p2.linear_acceleration == Vec3.new(0.0, 1.0, 0.0)
      assert p2.angular_velocity == Vec3.new(0.0, 1.0, 0.0)
    end

    test "ignores unknown messages", %{state: state} do
      reject(&BB.publish/3)
      assert {:noreply, ^state} = Sensor.handle_info(:something_else, state)
    end
  end

  describe "handle_options/2" do
    defp polling_state(opts_overrides \\ []) do
      opts = Map.new(polling_opts(opts_overrides))

      %{
        bb: default_bb_context(),
        bmi: fake_bmi(),
        mode: :polling,
        opts: opts,
        publish_interval_ms: hertz_to_ms(opts.publish_rate)
      }
    end

    defp interrupt_state(opts_overrides \\ []) do
      opts = Map.new(interrupt_opts(opts_overrides))

      %{
        bb: default_bb_context(),
        bmi: fake_bmi(),
        mode: :interrupt,
        opts: opts,
        sampler: :fake_sampler_pid
      }
    end

    defp hertz_to_ms(rate) do
      rate
      |> Unit.convert!("hertz")
      |> Units.extract_float()
      |> then(&round(1000 / &1))
    end

    test "recomputes publish_interval_ms when publish_rate changes" do
      state = polling_state()
      stub(BMI323, :configure_accelerometer, fn bmi, _ -> {:ok, bmi} end)
      stub(BMI323, :configure_gyroscope, fn bmi, _ -> {:ok, bmi} end)
      new_opts = polling_opts(publish_rate: ~u(50 hertz))

      assert {:ok, new_state} = Sensor.handle_options(new_opts, state)
      assert new_state.publish_interval_ms == 20
      assert new_state.bmi == state.bmi
    end

    test "applies accelerometer range / odr / mode changes live" do
      state = polling_state()
      stub(BMI323, :configure_gyroscope, fn bmi, _ -> {:ok, bmi} end)
      test_pid = self()

      expect(BMI323, :configure_accelerometer, fn bmi, opts ->
        send(test_pid, {:accel_opts, opts})
        {:ok, bmi}
      end)

      new_opts =
        polling_opts(
          accelerometer_range: 4,
          accelerometer_odr: 400,
          accelerometer_mode: :high_performance
        )

      assert {:ok, _} = Sensor.handle_options(new_opts, state)

      assert_receive {:accel_opts, opts}
      assert opts[:range] == 4
      assert opts[:odr] == 400
      assert opts[:mode] == :high_performance
    end

    test "applies gyroscope range / odr / mode changes live" do
      state = polling_state()
      stub(BMI323, :configure_accelerometer, fn bmi, _ -> {:ok, bmi} end)
      test_pid = self()

      expect(BMI323, :configure_gyroscope, fn bmi, opts ->
        send(test_pid, {:gyro_opts, opts})
        {:ok, bmi}
      end)

      new_opts =
        polling_opts(
          gyroscope_range: 500,
          gyroscope_odr: 100,
          gyroscope_mode: :low_power
        )

      assert {:ok, _} = Sensor.handle_options(new_opts, state)

      assert_receive {:gyro_opts, opts}
      assert opts[:range] == 500
      assert opts[:odr] == 100
      assert opts[:mode] == :low_power
    end

    test "does not call configure_* when only publish_rate changes" do
      state = polling_state()
      reject(&BMI323.configure_accelerometer/2)
      reject(&BMI323.configure_gyroscope/2)

      assert {:ok, _} = Sensor.handle_options(polling_opts(publish_rate: ~u(50 hertz)), state)
    end

    test "stops on configure_accelerometer failure" do
      state = polling_state()
      stub(BMI323, :configure_gyroscope, fn bmi, _ -> {:ok, bmi} end)
      expect(BMI323, :configure_accelerometer, fn _, _ -> {:error, :bad_range} end)

      assert {:stop, :bad_range} =
               Sensor.handle_options(polling_opts(accelerometer_range: 4), state)
    end

    test "stops with :reconfigure when mode changes" do
      state = polling_state()

      assert {:stop, :reconfigure} =
               Sensor.handle_options(interrupt_opts(), state)
    end

    test "stops with :reconfigure when bus changes" do
      state = polling_state()

      assert {:stop, :reconfigure} =
               Sensor.handle_options(polling_opts(bus: "i2c-9"), state)
    end

    test "stops with :reconfigure when address changes" do
      state = polling_state()

      assert {:stop, :reconfigure} =
               Sensor.handle_options(polling_opts(address: 0x69), state)
    end

    test "stops with :reconfigure when int1_pin changes (interrupt mode)" do
      state = interrupt_state()

      assert {:stop, :reconfigure} =
               Sensor.handle_options(interrupt_opts(int1_pin: 22), state)
    end

    test "stops with :reconfigure when watermark_frames changes (interrupt mode)" do
      state = interrupt_state()

      assert {:stop, :reconfigure} =
               Sensor.handle_options(interrupt_opts(watermark_frames: 16), state)
    end
  end
end
