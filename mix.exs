# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Sensor.BMI323.MixProject do
  use Mix.Project

  @moduledoc """
  Beam Bots integration for the Bosch BMI323 6-DoF IMU over I2C.
  """

  @version "0.1.3"

  def project do
    [
      aliases: aliases(),
      app: :bb_sensor_bmi323,
      consolidate_protocols: Mix.env() == :prod,
      deps: deps(),
      description: @moduledoc,
      dialyzer: dialyzer(),
      docs: docs(),
      elixir: "~> 1.19",
      elixirc_paths: elixirc_paths(Mix.env()),
      package: package(),
      start_permanent: Mix.env() == :prod,
      version: @version
    ]
  end

  defp dialyzer do
    [
      plt_add_apps: [:mix]
    ]
  end

  defp package do
    [
      maintainers: ["James Harton <james@harton.nz>"],
      licenses: ["Apache-2.0"],
      files: ~w(lib .formatter.exs mix.exs README* CHANGELOG* LICENSE* usage-rules.md),
      links: %{
        "Source" => "https://github.com/beam-bots/bb_sensor_bmi323",
        "Sponsor" => "https://github.com/sponsors/jimsynz"
      }
    ]
  end

  def application do
    [
      extra_applications: [:logger]
    ]
  end

  defp docs do
    [
      main: "readme",
      logo: "assets/logo.png",
      extras:
        ["README.md", "CHANGELOG.md"]
        |> Enum.concat(Path.wildcard("documentation/**/*.{md,livemd,cheatmd}")),
      groups_for_extras: [
        Tutorials: ~r/tutorials\//
      ],
      source_ref: "main",
      source_url: "https://github.com/beam-bots/bb_sensor_bmi323"
    ]
  end

  defp aliases, do: []

  defp bb_dep(default) do
    case System.get_env("BB_VERSION") do
      nil -> default
      "local" -> [path: "../bb", override: true]
      "main" -> [git: "https://github.com/beam-bots/bb.git", override: true]
      version -> "~> #{version}"
    end
  end

  defp deps do
    [
      {:bb, bb_dep("~> 0.20")},
      {:bmi323, "~> 0.1"},
      {:circuits_gpio, "~> 2.0"},
      {:circuits_i2c, "~> 2.0"},
      {:wafer, "~> 1.0"},

      # dev/test
      {:credo, "~> 1.7", only: [:dev, :test], runtime: false},
      {:dialyxir, "~> 1.4", only: [:dev, :test], runtime: false},
      {:doctor, "~> 0.23", only: [:dev, :test], runtime: false},
      {:ex_check, "~> 0.16", only: [:dev, :test], runtime: false},
      {:ex_doc, ">= 0.0.0", only: [:dev, :test], runtime: false},
      {:git_ops, "~> 2.9", only: [:dev, :test], runtime: false},
      {:igniter, "~> 0.6", only: [:dev, :test], runtime: false},
      {:mimic, "~> 2.2", only: :test, runtime: false},
      {:mix_audit, "~> 2.1", only: [:dev, :test], runtime: false},
      {:usage_rules, "~> 1.2", only: [:dev], runtime: false}
    ]
  end

  defp elixirc_paths(env) when env in [:dev, :test], do: ["lib", "test/support"]
  defp elixirc_paths(_), do: ["lib"]
end
