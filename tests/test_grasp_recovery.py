"""Regression protection for v1.7 reset and grasp collision model."""
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from phantomarm.controller import Controller
from phantomarm.kinematics import Pose, inverse_kinematics, forward_kinematics

XML = Path(__file__).resolve().parents[1] / 'assets' / 'phantom_arm_grasp.xml'


def _tree():
    return ET.parse(XML).getroot()


def _can_collide(a, b):
    ca, aa = int(a.get('contype', 0)), int(a.get('conaffinity', 0))
    cb, ab = int(b.get('contype', 0)), int(b.get('conaffinity', 0))
    return bool((ca & ab) or (cb & aa))


def test_reset_uses_safe_height_and_neutral_xy():
    c = Controller()
    c.nudge_xy(dx=.1, dy=.1)
    c.set_height(.13)
    c.reset_manual_target(height=.43)
    assert c.last_target == Pose(.38, 0, .43)
    assert forward_kinematics(inverse_kinematics(c.last_target)).z == pytest.approx(.43)


def test_reset_rejects_bad_heights():
    with pytest.raises(ValueError):
        Controller().reset_manual_target(height=.9)


def test_cube_stand_gripper_collision_filtering():
    root = _tree()
    cube = root.find(".//geom[@name='cube_collision']")
    stand = root.find(".//geom[@name='pickup_pedestal']")
    finger = root.find(".//body[@name='left_finger']/geom")
    assert _can_collide(cube, stand), 'cube must rest on stand'
    assert _can_collide(cube, finger), 'finger must actually grip cube'
    assert _can_collide(finger, stand), 'stand MUST block finger, not let it pass through'


def test_hover_is_above_cube_and_pads_clear_stand_on_approach():
    root = _tree()
    cube = root.find(".//geom[@name='cube_collision']")
    cube_body = root.find(".//body[@name='pickup_cube']")
    stand = root.find(".//geom[@name='pickup_pedestal']")
    left = root.find(".//body[@name='left_finger']/geom")
    cube_top = float(cube_body.get('pos').split()[2]) + float(cube.get('size').split()[2])
    stand_top = float(stand.get('pos').split()[2]) + float(stand.get('size').split()[2])
    pad_z_offset = float(left.get('pos').split()[2])
    pad_halfheight = float(left.get('size').split()[2])
    assert .43 + pad_z_offset - pad_halfheight > cube_top + .02
    assert .355 + pad_z_offset - pad_halfheight > stand_top


def test_new_diagnostic_script_present():
    assert (XML.parent.parent/'scripts'/'run_grasp_diagnostic.bat').exists()
