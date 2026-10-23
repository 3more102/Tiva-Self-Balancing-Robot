#!/usr/bin/env python3
"""Deterministic, standard-library-only *theoretical* pitch simulation.

NOT hardware data; NOT an identified full two-wheel robot.
Model: J*theta_ddot = m*g*l*sin(theta) - b*theta_dot - Ku*u.
Motor u represents an idealized signed effective pitch torque command:
it does not include wheels, motors, gearboxes, current limits, or delay.
The illustrative stabilizing PD gains here are NOT Tiva firmware gains.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class PitchParameters:
    m_kg: float = 0.65
    l_m: float = 0.10
    j_kgm2: float = 0.012
    b_nms_rad: float = 0.010
    ku_nm_per_command: float = 0.20
    g_m_s2: float = 9.81
    kp_per_degree: float = 0.09
    kd_per_degree_s: float = 0.008
    normalized_u_limit: float = 0.70
    dt_s: float = 0.005
    horizon_s: float = 8.0
    max_tilt_deg: float = 45.0


def clamp(v: float, lower: float, upper: float) -> float:
    return max(lower, min(v, upper))


def derivative(theta_rad: float, omega_rad_s: float,
               u: float, p: PitchParameters) -> tuple[float, float]:
    return (omega_rad_s,
            (p.m_kg * p.g_m_s2 * p.l_m * math.sin(theta_rad) -
             p.b_nms_rad * omega_rad_s - p.ku_nm_per_command * u) / p.j_kgm2)


def rk4(theta: float, omega: float, u: float, h: float,
        p: PitchParameters) -> tuple[float, float]:
    """One zero-order-held input step using RK4 integration."""
    a1, b1 = derivative(theta, omega, u, p)
    a2, b2 = derivative(theta + h*a1/2, omega + h*b1/2, u, p)
    a3, b3 = derivative(theta + h*a2/2, omega + h*b2/2, u, p)
    a4, b4 = derivative(theta + h*a3, omega + h*b3, u, p)
    return (theta + h*(a1 + 2*a2 + 2*a3 + a4)/6,
            omega + h*(b1 + 2*b2 + 2*b3 + b4)/6)


def simulate(initial_angle_deg: float = 4.0, enabled: bool = True,
             p: PitchParameters = PitchParameters()) -> tuple[list[dict], dict]:
    if not math.isfinite(initial_angle_deg):
        raise ValueError("initial angle must be finite")
    if any(not math.isfinite(v) for v in asdict(p).values()):
        raise ValueError("non-finite plant parameter")
    if min(p.m_kg, p.l_m, p.j_kgm2, p.ku_nm_per_command,
           p.dt_s, p.horizon_s, p.normalized_u_limit,
           p.max_tilt_deg) <= 0:
        raise ValueError("positive mass, inertia, geometry and sampling required")

    theta = math.radians(initial_angle_deg)
    omega = 0.0
    rows: list[dict] = []
    total = int(round(p.horizon_s/p.dt_s))
    trip = False
    for k in range(total+1):
        theta_deg = math.degrees(theta)
        omega_deg_s = math.degrees(omega)
        u = clamp(p.kp_per_degree*theta_deg + p.kd_per_degree_s*omega_deg_s,
                  -p.normalized_u_limit, p.normalized_u_limit) if enabled else 0.0
        rows.append({"time_s": round(k*p.dt_s, 9),
                     "angle_deg": theta_deg,
                     "gyro_x_dps": omega_deg_s,
                     "command": u,
                     "data_origin": "THEORETICAL_SIMULATION_NOT_MEASURED"})
        if abs(theta_deg) >= p.max_tilt_deg:
            trip = True
            break
        if k < total:
            theta, omega = rk4(theta, omega, u, p.dt_s, p)

    rms_angle = math.sqrt(sum(r["angle_deg"] ** 2 for r in rows)/len(rows))
    saturated = sum(abs(r["command"]) >= p.normalized_u_limit - 1e-8 for r in rows)
    metrics = {
        "data_origin": "THEORETICAL_SIMULATION_NOT_MEASURED",
        "controller": "idealized PD" if enabled else "open loop",
        "initial_angle_deg": initial_angle_deg,
        "simulated_duration_s": rows[-1]["time_s"],
        "sample_count": len(rows),
        "final_angle_deg": rows[-1]["angle_deg"],
        "rms_pitch_deg": rms_angle,
        "max_abs_pitch_deg": max(abs(r["angle_deg"]) for r in rows),
        "saturation_sample_fraction": saturated/len(rows),
        "terminated_at_model_tilt_limit": trip,
        "parameters": asdict(p),
        "limitations": "Pitch-only ideal torque; no wheel/motor dynamics, "
                       "sensor noise, battery variation, real-time jitter, or real trials."
    }
    return rows, metrics


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--angle", type=float, default=4.0,
                        help="illustrative starting angle in degrees")
    parser.add_argument("--open-loop", action="store_true",
                        help="disable virtual correcting torque")
    parser.add_argument("--output", type=Path,
                        help="optional destination for simulated CSV trace")
    args = parser.parse_args()
    rows, metrics = simulate(args.angle, not args.open_loop)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", newline="", encoding="utf8") as stream:
            w = csv.DictWriter(stream, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    print(json.dumps(metrics, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
