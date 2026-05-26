# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.BbSensorBmi323.Install do
    @shortdoc "Installs BB.Sensor.BMI323 into a robot"
    @moduledoc """
    #{@shortdoc}

    Adds a `:config.:bmi323` param group with `bus`, `address`, and
    `int1_pin`, sets the bus name on the robot's child spec in your
    application module, and imports `bb_sensor_bmi323` into your
    formatter.

    The sensor itself lives on a specific link — the installer can't
    guess which one, so it prints a snippet for you to paste into the
    topology.

    ## Example

    ```bash
    mix igniter.install bb_sensor_bmi323
    mix igniter.install bb_sensor_bmi323 --bus ftdi-3:17-i2c
    ```

    ## Options

    * `--robot` - The robot module (defaults to `{AppPrefix}.Robot`).
    * `--bus` - The I2C bus name (default `i2c-1`).
    """

    use Igniter.Mix.Task

    alias Igniter.Project.Formatter

    @param_group :bmi323
    @default_bus "i2c-1"

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{
        schema: [
          robot: :string,
          bus: :string
        ],
        aliases: [r: :robot]
      }
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      options = igniter.args.options
      robot_module = BB.Igniter.robot_module(igniter)
      bus = Keyword.get(options, :bus, @default_bus)

      igniter
      |> Formatter.import_dep(:bb_sensor_bmi323)
      |> BB.Igniter.add_param_group(robot_module, [:config, @param_group], param_group_body())
      |> BB.Igniter.set_robot_opts(robot_module,
        params: [config: [{@param_group, [bus: bus]}]]
      )
      |> Igniter.add_notice(topology_snippet())
    end

    defp param_group_body do
      """
      param :bus, type: :string, doc: "I2C bus name (e.g. \\"i2c-1\\")"

      param :address,
        type: :integer,
        default: 0x68,
        doc: "I2C address of the BMI323"

      param :int1_pin,
        type: {:or, [:non_neg_integer, nil]},
        default: nil,
        doc: "GPIO pin number wired to BMI323 INT1 (required for :interrupt mode)"
      """
    end

    defp topology_snippet do
      """
      bb_sensor_bmi323: add a sensor to whichever link the IMU is mounted
      on. Polling-mode example:

          link :base do
            sensor :imu, {BB.Sensor.BMI323,
              bus: param([:config, :bmi323, :bus]),
              address: param([:config, :bmi323, :address]),
              mode: :polling,
              accelerometer_range: 8,
              accelerometer_odr: 200,
              gyroscope_range: 2000,
              gyroscope_odr: 200,
              publish_rate: ~u(100 hertz)
            }
          end

      For interrupt mode, wire INT1 to a GPIO and reference it via the
      param:

          sensor :imu, {BB.Sensor.BMI323,
            bus: param([:config, :bmi323, :bus]),
            address: param([:config, :bmi323, :address]),
            mode: :interrupt,
            int1_pin: param([:config, :bmi323, :int1_pin]),
            accelerometer_range: 8,
            accelerometer_odr: 800,
            gyroscope_range: 2000,
            gyroscope_odr: 800,
            watermark_frames: 8
          }
      """
    end
  end
else
  defmodule Mix.Tasks.BbSensorBmi323.Install do
    @shortdoc "Installs BB.Sensor.BMI323 into a robot"
    @moduledoc false
    use Mix.Task

    def run(_argv) do
      Mix.shell().error("""
      The bb_sensor_bmi323.install task requires igniter.

          mix igniter.install bb_sensor_bmi323
      """)

      exit({:shutdown, 1})
    end
  end
end
