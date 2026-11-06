"""Viewer shortcut regression checks for PhantomArm controller keys."""
from pathlib import Path
import xml.etree.ElementTree as ET
from phantomarm.viewer_controls import action_for_key


def test_custom_controls_are_distinct_from_mujoco_reserved_keys():
    # These are all MuJoCo viewer shortcut keys, not app input.
    for key in '012345ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz':
        assert action_for_key(ord(key)) is None
    expected={54:'gripper',55:'forward',56:'backward',57:'estop',297:'reset',
              266:'reset_cube',296:'reset_cube',298:'home',299:'live',300:'calibrate',301:'quit',
              32:'reset',262:'right',263:'left',264:'lower',265:'raise',
              268:'home',269:'live',261:'reset_cube',
              257:'calibrate',335:'calibrate'}
    for key,action in expected.items():
        assert action_for_key(key)==action
    for bad in [-1, 290,291,292,293,294,295,ord('0'),ord('5')]:
        assert action_for_key(bad) is None


def test_floor_and_arm_collision_masks_are_compatible():
    root=Path(__file__).resolve().parent.parent
    xml=ET.parse(root/'assets'/'phantom_arm.xml').getroot()
    floor=xml.find(".//geom[@name='floor']")
    assert floor.attrib['contype']=='1' and floor.attrib['conaffinity']=='2'
    # At least the forearm, upper arm and wrist can collide with the ground,
    # without forcing the robot to collide with itself.
    robot_geoms=[g for g in xml.findall('.//body//geom') if g.get('contype')=='2']
    assert len(robot_geoms)>=5
    for geom in robot_geoms:
        assert int(geom.get('contype')) & int(floor.get('conaffinity'))
        assert geom.get('conaffinity')=='0'
    marker=xml.find(".//site[@name='workspace_goal']")
    assert marker.get('type')=='box'


def test_laptop_shortcuts_have_same_actions_as_function_keys():
    assert action_for_key(32) == action_for_key(297) == 'reset'
    assert action_for_key(268) == action_for_key(298) == 'home'
    assert action_for_key(259) is None  # Backspace is MuJoCo native full-scene reset
    assert action_for_key(269) == action_for_key(299) == 'live'
    assert action_for_key(266) == action_for_key(261) == action_for_key(296) == 'reset_cube'
    assert action_for_key(257) == action_for_key(335) == action_for_key(300) == 'calibrate'

def test_punctuation_render_shortcuts_not_hijacked():
    # MuJoCo uses these for render flags or camera/speed. We don't steal them.
    for key in (ord(','),ord('.'),ord('['),ord(']'),ord('-'),ord('='),ord('\\'),ord(';'),ord('`'),ord('/')):
        assert action_for_key(key) is None
