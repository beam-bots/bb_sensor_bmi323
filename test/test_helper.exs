# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

Application.ensure_all_started(:mimic)

ExUnit.start()

Mimic.copy(BB)
Mimic.copy(BMI323)
Mimic.copy(BMI323.Sampler)
Mimic.copy(Wafer.Driver.Circuits.GPIO)
Mimic.copy(Wafer.Driver.Circuits.I2C)
