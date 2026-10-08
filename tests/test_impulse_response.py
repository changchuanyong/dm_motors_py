"""无硬件验证脉冲响应示例的判定逻辑。"""

import runpy
import unittest
from pathlib import Path


EXAMPLE = (
    Path(__file__).resolve().parents[1]
    / "examples"
    / "DM_Motor_Impulse_Response.py"
)


class ImpulseResponseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.namespace = runpy.run_path(str(EXAMPLE), run_name="impulse_import")

    def test_response_passes_when_motor_moves_and_settles(self):
        samples = [
            {"time": 0.01, "position": 0.0, "velocity": 0.0},
            {"time": 0.20, "position": 0.05, "velocity": 0.5},
            {"time": 0.80, "position": 0.09, "velocity": 0.08},
            {"time": 1.00, "position": 0.091, "velocity": 0.04},
        ]
        result = self.namespace["analyze_response"](
            samples,
            initial_position=0.0,
            min_position_change=0.01,
            max_final_speed=0.1,
            settle_window=0.3,
        )
        self.assertTrue(result["moved"])
        self.assertTrue(result["stopped"])
        self.assertTrue(result["passed"])
        self.assertAlmostEqual(result["max_displacement"], 0.091)

    def test_response_fails_when_it_does_not_settle(self):
        samples = [
            {"time": 0.1, "position": 0.0, "velocity": 0.0},
            {"time": 0.8, "position": 0.02, "velocity": 0.4},
            {"time": 1.0, "position": 0.03, "velocity": 0.35},
        ]
        result = self.namespace["analyze_response"](
            samples,
            initial_position=0.0,
            min_position_change=0.01,
            max_final_speed=0.1,
            settle_window=0.5,
        )
        self.assertTrue(result["moved"])
        self.assertFalse(result["stopped"])
        self.assertFalse(result["passed"])

    def test_empty_feedback_is_rejected(self):
        with self.assertRaises(ValueError):
            self.namespace["analyze_response"]([], initial_position=0.0)


if __name__ == "__main__":
    unittest.main()
