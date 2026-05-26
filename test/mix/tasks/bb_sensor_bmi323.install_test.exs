# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule Mix.Tasks.BbSensorBmi323.InstallTest do
  use ExUnit.Case
  import Igniter.Test

  @moduletag :igniter

  defp project_with_robot do
    test_project()
    |> Igniter.compose_task("bb.install")
    |> apply_igniter!()
  end

  describe "parameters group" do
    test "adds a :config.:bmi323 param group with bus, address, and int1_pin" do
      project_with_robot()
      |> Igniter.compose_task("bb_sensor_bmi323.install")
      |> assert_has_patch("lib/test/robot.ex", """
      + |    group :config do
      + |      group :bmi323 do
      + |        param(:bus, type: :string, doc: "I2C bus name (e.g. \\"i2c-1\\")")
      """)
    end
  end

  describe "application module" do
    test "sets the bus name on the robot child spec" do
      project_with_robot()
      |> Igniter.compose_task("bb_sensor_bmi323.install")
      |> assert_has_patch("lib/test/application.ex", ~s'''
      + |    children = [{Test.Robot, [params: [config: [bmi323: [bus: "i2c-1"]]]]}]
      ''')
    end

    test "honours a custom --bus option" do
      project_with_robot()
      |> Igniter.compose_task("bb_sensor_bmi323.install", ["--bus", "ftdi-3:17-i2c"])
      |> assert_has_patch("lib/test/application.ex", ~s'''
      + |    children = [{Test.Robot, [params: [config: [bmi323: [bus: "ftdi-3:17-i2c"]]]]}]
      ''')
    end
  end

  describe "formatter" do
    test "imports bb_sensor_bmi323 into .formatter.exs" do
      project_with_robot()
      |> Igniter.compose_task("bb_sensor_bmi323.install")
      |> assert_has_patch(".formatter.exs", """
      + |  import_deps: [:bb_sensor_bmi323, :bb]
      """)
    end
  end

  describe "notice" do
    test "prints a topology snippet for the user to paste" do
      project_with_robot()
      |> Igniter.compose_task("bb_sensor_bmi323.install")
      |> assert_has_notice(&String.contains?(&1, "BB.Sensor.BMI323"))
    end
  end

  describe "idempotency" do
    test "running twice produces no further changes" do
      project_with_robot()
      |> Igniter.compose_task("bb_sensor_bmi323.install")
      |> apply_igniter!()
      |> Igniter.compose_task("bb_sensor_bmi323.install")
      |> assert_unchanged()
    end
  end
end
