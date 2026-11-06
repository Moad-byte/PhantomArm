"""Regression tests replacing v1.9 assumptions about the unstable narrow stand."""
from pathlib import Path
import xml.etree.ElementTree as ET

r=ET.parse(Path(__file__).resolve().parents[1]/'assets'/'phantom_arm_grasp.xml').getroot()

def get(name):
    elem=r.find(f'.//geom[@name="{name}"]')
    assert elem is not None
    return elem

def test_stable_stand_exceeds_cube_footprint():
    stand=get('pickup_pedestal')
    cube=get('cube_collision')
    sx, sy, _ = map(float, stand.get('size').split())
    cx, cy, _ = map(float, cube.get('size').split())
    assert sx > cx and sy > cy

def test_no_under_grasp_lips_clashing_with_stand():
    assert r.find('.//geom[@name="left_support_lip"]') is None
    assert r.find('.//geom[@name="right_support_lip"]') is None

def test_free_moving_cube_and_collidable_platform():
    assert r.find('.//body[@name="pickup_cube"]/freejoint') is not None
    cube=get('cube_collision'); stand=get('pickup_pedestal')
    assert (int(cube.get('contype')) & int(stand.get('conaffinity'))) or (int(stand.get('contype')) & int(cube.get('conaffinity')))
