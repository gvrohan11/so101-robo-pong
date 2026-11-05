import unittest
import json
from pathlib import Path
from toss_motion import plan, execute, JOINTS

class TossTests(unittest.TestCase):
    def setUp(self):
        self.c=json.loads(Path(__file__).with_name("toss.example.json").read_text())
    def test_endpoints_and_fixed_axes(self):
        samples=plan(self.c)
        self.assertEqual(samples[0].positions, self.c["poses"]["ready"])
        self.assertEqual(samples[-1].positions, self.c["poses"]["ready"])
        self.assertTrue(all(b.time_s>a.time_s for a,b in zip(samples,samples[1:])))
        self.assertTrue(all(s.positions["gripper"]==30 for s in samples))
    def test_release_stays_open_on_return(self):
        samples=plan(self.c,release=True)
        openings=[s.positions["gripper"] for s in samples]
        self.assertEqual(openings[-1],80)
        self.assertEqual(openings,sorted(openings))
    def test_overspeed_and_invalid_geometry_rejected(self):
        with self.assertRaises(ValueError): plan(self.c,speed=3)
        self.c["poses"]["windup"]["elbow_flex"]=100
        with self.assertRaises(ValueError): plan(self.c)
    def test_no_commands_when_start_mismatches(self):
        commands=[]
        with self.assertRaises(RuntimeError):
            execute(plan(self.c),lambda:{j:100 for j in JOINTS},commands.append,12,.1,lambda e:None)
        self.assertEqual(commands,[])

if __name__=="__main__": unittest.main()
