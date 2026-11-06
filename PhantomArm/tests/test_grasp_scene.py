from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

ROOT = Path(__file__).resolve().parent.parent
SCENE = ROOT / 'assets' / 'phantom_arm_grasp.xml'

def test_scene_has_a_real_free_joint_cube():
    root = ET.parse(SCENE).getroot()
    cube = root.find(".//body[@name='pickup_cube']")
    assert cube is not None
    assert cube.find("freejoint[@name='cube_free']") is not None
    geom = cube.find("geom[@name='cube_collision']")
    assert geom is not None
    assert geom.get('type') == 'box'
    assert int(geom.get('contype')) & 4
    assert int(geom.get('conaffinity')) & 2

def test_two_real_stands_and_collision_enabled_fingers():
    root = ET.parse(SCENE).getroot()
    assert root.find(".//geom[@name='pickup_pedestal']") is not None
    assert root.find(".//geom[@name='drop_pedestal']") is not None
    for body_name in ('left_finger','right_finger'):
        finger = root.find(f".//body[@name='{body_name}']/geom")
        assert finger is not None
        assert int(finger.get('contype')) & 2
        assert int(finger.get('conaffinity')) & 4  # cube collision group

def test_mujoco_compiles_grasp_scene_when_available():
    mujoco = pytest.importorskip('mujoco')
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    assert m.nu == 6
    assert m.nq == 13   # 6 robot coordinates, 7 free-joint cube coordinates
    assert m.nv == 12
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'pickup_cube')
    assert bid >= 0
    assert 0.34 < d.xpos[bid][2] < 0.37


def test_gripper_pad_area_and_motor_strength():
    root = ET.parse(SCENE).getroot()
    for finger_name in ('left_finger','right_finger'):
        geom = root.find(f".//body[@name='{finger_name}']/geom")
        assert float(geom.get('size').split()[2]) >= 0.023  # comparable height to cube
        assert float(geom.get('friction').split()[0]) >= 3.0
    for motor in ('motor_left','motor_right'):
        node = root.find(f".//actuator/position[@name='{motor}']")
        assert node is not None
        assert float(node.get('kp')) >= 200
        assert node.get('forcerange')


def test_explicit_reset_preserves_physics_based_cube():
    from phantomarm.viewer_controls import action_for_key
    assert action_for_key(266) == 'reset_cube'  # page up
    assert action_for_key(261) == 'reset_cube'  # Delete
    assert action_for_key(259) is None  # NEVER Backspace: native MuJoCo world reset
    assert action_for_key(296) == 'reset_cube'  # F7
    root = ET.parse(SCENE).getroot()
    cube = root.find(".//body[@name='pickup_cube']")
    assert cube.find('freejoint') is not None
    # No weld constraints or equality that would automatically attach the cube.
    assert root.find('equality') is None


def test_reset_only_moves_cube_when_mujoco_installed():
    mujoco = pytest.importorskip('mujoco')
    from phantomarm.grasp import restore_cube_only
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    d = mujoco.MjData(m)
    d.qpos[:6] = (.3,.2,-.4,.25,.012,.014)
    d.ctrl[:6] = (.3,.2,-.4,.25,.012,.014)
    d.qpos[6:9] = (.9,.8,.02)
    original_qpos = d.qpos[:6].copy()
    restore_cube_only(mujoco,m,d)
    assert list(d.qpos[:6]) == list(original_qpos)
    cube = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'pickup_cube')
    assert abs(float(d.xpos[cube][0]) - .43) < 1e-4
    assert abs(float(d.xpos[cube][1])) < 1e-4
