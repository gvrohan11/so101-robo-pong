import json, unittest
from pathlib import Path
import main
from toss_params import TossParams, NAMES

class TossTests(unittest.TestCase):
    def setUp(self):
        self.cal = json.loads((Path(main.HERE)/"calibration/my_awesome_follower_arm.json").read_text())
        self.lim = main.limits_from_calibration(self.cal, main.LIMIT_MARGIN_DEG)
    def plan(self, p=None, slow=False):
        return main.build_plan(main.START_POSE, main.START_POSE, self.lim, slow, p or TossParams(), self.cal)
    def test_moves_match_params(self):
        p = TossParams()
        s, w, f, _, _ = self.plan(p)
        for j, d in p.windup_move().items():
            self.assertAlmostEqual(w[j] - main.START_POSE[j], d, places=6)
        for j, d in p.release_move().items():
            self.assertAlmostEqual(f[j] - w[j], d, places=6)
    def test_returns_to_start_and_fixed_joints(self):
        for slow in (False, True):
            s, *_ = self.plan(slow=slow)
            for j in main.JOINTS:
                self.assertAlmostEqual(s[-1].q[j], main.START_POSE[j], places=6)
            self.assertTrue(all(b.t > a.t for a, b in zip(s, s[1:])))
            for j in ("shoulder_pan", "wrist_roll"):
                self.assertTrue(all(x.q[j] == main.START_POSE[j] for x in s))
    def test_gripper(self):
        p = TossParams()
        s, w, f, _, info = self.plan(p)
        pre = [x for x in s if x.phase in ("approach", "windup")]
        self.assertTrue(all(x.q["gripper"] == main.GRIPPER_CLOSED for x in pre[1:]))
        self.assertAlmostEqual(f["gripper"] - main.GRIPPER_CLOSED,
                               p.gripper_move * main.gripper_pct_per_deg(self.cal), places=6)
        self.assertEqual(s[-1].q["gripper"], main.GRIPPER_CLOSED)
    def test_vector_roundtrip_and_limits(self):
        p = TossParams()
        self.assertEqual(TossParams.from_vector(p.to_vector()), p)
        self.assertEqual(len(NAMES), 14)
        with self.assertRaises(ValueError):
            self.plan(TossParams(release_move_wrist=+30))   # past wrist limit
        with self.assertRaises(ValueError):
            self.plan(TossParams(release_speed_elbow=400))  # over speed cap

if __name__ == "__main__":
    unittest.main()
