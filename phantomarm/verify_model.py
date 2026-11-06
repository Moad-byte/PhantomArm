"""Compile MJCF and compare computed FK against MuJoCo sensor world positions."""
from pathlib import Path
import math
from .kinematics import Pose,inverse_kinematics,forward_kinematics

def main():
    import mujoco
    path=Path(__file__).resolve().parents[1]/'assets'/'phantom_arm.xml'
    m=mujoco.MjModel.from_xml_path(str(path))
    d=mujoco.MjData(m)
    id=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'tool_tip')
    assert id>=0 and m.nu==6
    targets=[Pose(.38,0,.35),Pose(.34,.10,.41),Pose(.44,-.08,.31),Pose(.31,.04,.46)]
    for target in targets:
        q=inverse_kinematics(target)
        d.qpos[:4]=q
        mujoco.mj_forward(m,d)
        actual=tuple(float(v) for v in d.site_xpos[id])
        predicted=forward_kinematics(q)
        err=math.dist(actual,(target.x,target.y,target.z))
        print('target',target,'actual',tuple(round(n,4) for n in actual),'error',round(err,7),'m')
        assert err<1e-4, f'FK / MJCF mismatch: {err:.4f}'
        assert math.dist(actual,(predicted.x,predicted.y,predicted.z))<1e-4
    print('PASS: MJCF model compiles; 6 actuators; 4 IK/FK model checks')

if __name__=='__main__':main()
