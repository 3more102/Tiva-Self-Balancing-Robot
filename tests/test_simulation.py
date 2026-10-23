"""Unit/regression tests for theoretical pitch-only plant; NOT hardware validation."""
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation.run_pitch_sim import PitchParameters, simulate, rk4


class PitchSimulationTests(unittest.TestCase):
    def test_undisturbed_equilibrium(self):
        rows, metrics = simulate(0.0)
        self.assertEqual(len(rows), 1601)
        self.assertAlmostEqual(metrics["final_angle_deg"], 0.0, places=9)
        self.assertEqual(metrics["data_origin"], "THEORETICAL_SIMULATION_NOT_MEASURED")

    def test_illustrative_pd_stabilizes_four_degrees(self):
        rows, metrics = simulate(4.0)
        self.assertFalse(metrics["terminated_at_model_tilt_limit"])
        self.assertLess(abs(metrics["final_angle_deg"]), 0.1)
        self.assertLess(metrics["max_abs_pitch_deg"], 5.0)
        self.assertTrue(all(abs(row["command"]) <= 0.70000001 for row in rows))
        self.assertTrue(all(row["data_origin"].startswith("THEORETICAL") for row in rows))

    def test_uncontrolled_pendulum_falls_out_of_model_validity(self):
        rows, metrics = simulate(4.0, enabled=False)
        self.assertTrue(metrics["terminated_at_model_tilt_limit"])
        self.assertGreaterEqual(abs(metrics["final_angle_deg"]), 45.0)
        self.assertLess(rows[-1]["time_s"], 8.0)

    def test_rk4_dynamics_and_input_validation(self):
        p = PitchParameters()
        theta, rate = rk4(0.0, 0.0, 0.0, p.dt_s, p)
        self.assertEqual(theta, 0.0)
        self.assertEqual(rate, 0.0)
        with self.assertRaises(ValueError):
            simulate(float("nan"))
        with self.assertRaises(ValueError):
            simulate(2.0, p=replace(p, j_kgm2=0.0))


if __name__ == "__main__":
    unittest.main()
