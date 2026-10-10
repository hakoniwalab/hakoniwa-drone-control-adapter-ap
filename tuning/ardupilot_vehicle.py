# SPDX-License-Identifier: GPL-3.0-or-later
"""Vehicle-derived ArduPilot motor values, shared by the tuning input preparation and the export.

ArduPilot output chain (AP_MotorsMulticopter, AP_Motors_Thrust_Linearization): thrust fraction t
(0..1 of the thrust at MOT_SPIN_MAX) -> curve c with t = EXPO*c^2 + (1-EXPO)*c -> actuator
a = SPIN_MIN + (SPIN_MAX-SPIN_MIN)*c -> PWM = PWM_MIN + (PWM_MAX-PWM_MIN)*a. Drone PRO's SITL service
reads duty = (PWM-1000)/1000, so PWM 1000..2000 makes duty = a. MOT_THST_HOVER is the hover thrust as a
fraction of the thrust at MOT_SPIN_MAX; MOT_THST_EXPO is solved at the hover point, as PX4's
THR_MDL_FAC (hover point, no least squares). The tuning (MOT_THST_HOVER in the adapter configuration)
and the export use this one computation, so the tuned vehicle and the SITL vehicle have the same hover.
"""

from __future__ import annotations

import math
from typing import Any, Callable

PWM_MIN = 1000
PWM_MAX = 2000
SPIN_MIN = 0.15  # ArduPilot defaults (AP_MotorsMulticopter.h)
SPIN_MAX = 0.95


def motor_thrust_model(
    drone_config: dict[str, Any],
    controller_params: dict[str, float],
    compute_hover_thrust_fraction: Callable,
) -> dict[str, float]:
    """MOT_THST_HOVER and MOT_THST_EXPO from the Hakoniwa rotor model.

    compute_hover_thrust_fraction is Drone PRO's tuning.px4.conversion.native_params function
    (the rotor model's hover duty and maximum speed).
    """
    _, hover, _ = compute_hover_thrust_fraction(drone_config, controller_params)
    inputs = hover["inputs"]
    k, r, cq, d, v_bat = inputs["K"], inputs["R"], inputs["Cq"], inputs["D"], inputs["NominalVoltage"]
    omega_max = hover["omega_max_effective"]
    hover_duty = hover["hover_duty"]
    a_coef, b_coef = cq * r / k, k + d * r / k

    def thrust_at_duty(duty: float) -> float:  # relative to the thrust at omega_max
        omega = ((-b_coef + math.sqrt(b_coef * b_coef + 4.0 * a_coef * v_bat * duty)) / (2.0 * a_coef)
                 if a_coef > 0 else v_bat * duty / b_coef)
        return min(omega, omega_max) ** 2 / omega_max ** 2

    if not SPIN_MIN < hover_duty < SPIN_MAX:
        raise ValueError(f"hover duty {hover_duty:.3f} is outside MOT_SPIN_MIN..MOT_SPIN_MAX ({SPIN_MIN}..{SPIN_MAX})")
    thrust_hover = thrust_at_duty(hover_duty) / thrust_at_duty(SPIN_MAX)
    curve_hover = (hover_duty - SPIN_MIN) / (SPIN_MAX - SPIN_MIN)
    raw_expo = (thrust_hover - curve_hover) / (curve_hover * curve_hover - curve_hover)
    return {
        "hover_duty": hover_duty,
        "thrust_hover": thrust_hover,
        "expo": min(1.0, max(0.0, raw_expo)),
        "raw_expo": raw_expo,
    }
