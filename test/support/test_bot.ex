# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sensor.BMI323.TestBot do
  @moduledoc """
  Minimal robot for live-testing `BB.Sensor.BMI323` against real hardware.

  Not loaded by default — included in `test/support/` so it's compiled in
  the `:dev` and `:test` envs. Run it from IEx:

      iex> BB.Sensor.BMI323.TestBot.start_link([])
      iex> BB.subscribe(BB.Sensor.BMI323.TestBot, [:sensor, :base, :imu])
      iex> flush()
  """
  use BB
  import BB.Unit

  settings do
    name(:bb_sensor_bmi323_test_bot)
  end

  topology do
    link :base do
      sensor(
        :imu,
        {BB.Sensor.BMI323,
         bus: "i2c-1",
         address: 0x68,
         mode: :polling,
         accelerometer_range: 8,
         accelerometer_odr: 200,
         gyroscope_range: 2000,
         gyroscope_odr: 200,
         publish_rate: ~u(100 hertz)}
      )
    end
  end
end
