from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
xml=ET.parse(ROOT/'assets'/'phantom_arm_grasp.xml').getroot()

def test_pickup_is_stable_wider_than_cube():
    table=xml.find(".//geom[@name='pickup_pedestal']")
    cube=xml.find(".//geom[@name='cube_collision']")
    sy=float(table.get('size').split()[1]); cy=float(cube.get('size').split()[1]);
    assert sy>cy

def test_no_lips_hit_platform():
    assert xml.find(".//geom[@name='left_support_lip']") is None
    assert xml.find(".//geom[@name='right_support_lip']") is None

def test_console_cube_reset_does_not_call_global_reset():
    text=(ROOT/'phantomarm'/'grasp.py').read_text()
    func=text.split('def restore_cube_only')[1]
    assert 'mj_resetData(' not in func
    app=(ROOT/'phantomarm'/'app.py').read_text()
    assert 'terminal_commands.get_nowait()' in app
    assert 'type CUBE' in app
    assert 'robot_q_before' in app
