import unittest
from types import SimpleNamespace
from motor_startup import seed_targets

class Bus:
    def __init__(self,powered=0,bad_write=False):
        self.regs={'Present_Position':1750,'Goal_Position':0,'Torque_Enable':powered}
        self.writes=[]
        self.bad_write=bad_write
    def read(self,register,joint,normalize=False):return self.regs[register]
    def write(self,register,joint,value,normalize=False):
        self.writes.append((register,value,normalize))
        if not self.bad_write:self.regs[register]=value

class StartupTests(unittest.TestCase):
    def setUp(self):self.cal={'wrist_flex':SimpleNamespace(range_min=889,range_max=3099)}
    def test_raw_target_seeded_without_enabling(self):
        bus=Bus();seed_targets(bus,self.cal,['wrist_flex'])
        self.assertEqual(bus.writes,[('Goal_Position',1750,False)])
        self.assertEqual(bus.regs['Torque_Enable'],0)
    def test_failed_write_never_enables(self):
        bus=Bus(bad_write=True)
        with self.assertRaisesRegex(RuntimeError,'read-back'):seed_targets(bus,self.cal,['wrist_flex'])
        self.assertEqual(bus.regs['Torque_Enable'],0)
    def test_stale_powered_target_is_rejected_without_writes(self):
        bus=Bus(powered=1)
        with self.assertRaisesRegex(RuntimeError,'powered'):seed_targets(bus,self.cal,['wrist_flex'])
        self.assertEqual(bus.writes,[])
    def test_bad_encoder_refused_without_writes(self):
        bus=Bus();bus.regs['Present_Position']=0
        with self.assertRaises(RuntimeError):seed_targets(bus,self.cal,['wrist_flex'])
        self.assertEqual(bus.writes,[])
    def test_powered_hold_is_not_rewritten(self):
        bus=Bus(powered=1);bus.regs['Goal_Position']=1750
        seed_targets(bus,self.cal,['wrist_flex'])
        self.assertEqual(bus.writes,[])

if __name__=='__main__':unittest.main()
