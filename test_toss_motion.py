import json, unittest
from pathlib import Path
import main
from toss_motion import JOINTS, SWING_JOINTS

class TossTests(unittest.TestCase):
    def setUp(self):
        cal = json.loads((Path(main.HERE)/"calibration/my_awesome_follower_arm.json").read_text())
        self.lim = main.limits_from_calibration(cal, main.LIMIT_MARGIN_DEG)
    def test_plan_passes_and_returns_to_start(self):
        for slow in (False, True):
            s, w, f, _ = main.build_plan(main.START_POSE, main.START_POSE, self.lim, slow)
            for j in JOINTS:
                self.assertAlmostEqual(s[-1].q[j], main.START_POSE[j], places=6)
            self.assertTrue(all(b.t > a.t for a, b in zip(s, s[1:])))
    def test_windup_reverses_and_fixed_joints(self):
        s, w, f, _ = main.build_plan(main.START_POSE, main.START_POSE, self.lim, False)
        for j in SWING_JOINTS:
            self.assertLess((w[j]-main.START_POSE[j])*(f[j]-main.START_POSE[j]), 0)
        for j in ("shoulder_pan", "wrist_roll"):
            self.assertTrue(all(x.q[j] == main.START_POSE[j] for x in s))
    def test_gripper_closed_then_opens_in_release(self):
        for slow in (False, True):
            s, *_ = main.build_plan(main.START_POSE, main.START_POSE, self.lim, slow)
            pre = [x for x in s if x.phase in ("approach", "windup")]
            self.assertTrue(all(x.q["gripper"] == main.GRIPPER_CLOSED for x in pre[1:]))
            sw = [x.q["gripper"] for x in s if x.phase == "swing"]
            self.assertEqual(sw[0], main.GRIPPER_CLOSED)
            self.assertEqual(sw[-1], main.GRIPPER_OPEN_CMD)
            self.assertEqual(s[-1].q["gripper"], main.GRIPPER_CLOSED)
    def test_far_ratios_kept(self):
        _, w, f, _ = main.build_plan(main.START_POSE, main.START_POSE, self.lim, False)
        self.assertAlmostEqual(w["shoulder_lift"]-main.START_POSE["shoulder_lift"], main.SHOULDER_WINDUP_DEG)  # wrist has extra tuck

if __name__ == "__main__":
    unittest.main()
