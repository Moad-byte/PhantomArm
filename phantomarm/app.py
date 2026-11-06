from .cube_reset_hotkey import insert_just_pressed
"""PhantomArm simulation entry point.

Examples:
 python -m phantomarm.app --mode demo
 python -m phantomarm.app --mode demo --headless --duration 5
 python -m phantomarm.app --mode serial --port COM5 --record recordings/hardware.jsonl
 python -m phantomarm.app --mode replay --file recordings/sample_motion.jsonl
"""
import argparse
import math
import time
import queue
import threading
from pathlib import Path
from .controller import Controller,State
from .kinematics import Pose,inverse_kinematics,UnreachableTarget
from .motion import slew
from .sources import DemoSource,SerialSource,ReplaySource
from .recording import Recorder
from .diagnostics import SessionLog
from .viewer_controls import action_for_key
from .grasp import cube_spawn_is_clear, restore_cube_only
from .held_grip import HeldGrip
from .grasp_safety import clamp_grasp_target
from .windows_keys import held_gripper_keys

ROOT=Path(__file__).resolve().parent.parent
MODEL=ROOT/'assets'/'phantom_arm.xml'
GRASP_MODEL=ROOT/'assets'/'phantom_arm_grasp.xml'


def main(argv=None):
    ap=argparse.ArgumentParser(description='PhantomArm real-time 3D teleoperation')
    ap.add_argument('--mode', choices=('demo','serial','replay'),default='demo')
    ap.add_argument('--scene', choices=('standard','grasp'), default='standard',
                    help='grasp: movable cube and two real contact platforms; standard: original')
    ap.add_argument('--demo-pattern', choices=('still','sweep'),default='still',
                    help='still: independent gripper/height tests; sweep: automatic X/Y movement')
    ap.add_argument('--port',help='Windows COM port, e.g. COM5')
    ap.add_argument('--file',help='JSONL recording for --mode replay')
    ap.add_argument('--record',help='Write incoming data to JSONL')
    ap.add_argument('--log',help='CSV metrics destination')
    ap.add_argument('--headless',action='store_true',help='Run without interactive OpenGL viewer')
    ap.add_argument('--duration',type=float,default=0,help='Auto-stop after this many seconds; default until window closes')
    args=ap.parse_args(argv)
    if args.duration<0:ap.error('duration must be >=0')
    if args.mode=='serial' and not args.port:ap.error('--mode serial requires --port COMx')
    if args.mode=='replay' and not args.file:ap.error('--mode replay requires --file')
    if args.headless and args.duration==0:args.duration=5
    import mujoco
    import mujoco.viewer
    model=mujoco.MjModel.from_xml_path(str(GRASP_MODEL if args.scene=='grasp' else MODEL))
    data=mujoco.MjData(model)
    if model.nu!=6:raise RuntimeError('Robot model must expose six actuators')
    cube_body_id = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'pickup_cube') if args.scene=='grasp' else -1
    cube_geom_id = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'cube_collision') if args.scene=='grasp' else -1
    # The robot joints must come before the free-moving cube in MJCF.
    if args.scene=='grasp' and (model.nq!=13 or cube_geom_id<0):
        raise RuntimeError('Grasp scene has unexpected robot/cube joint layout')
    tip_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'tool_tip')
    goal_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'workspace_goal')
    grip_center_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'grip_center') if args.scene=='grasp' else -1
    if tip_id<0 or goal_id<0:raise RuntimeError('Robot sites missing')
    # In grasp mode start ABOVE the cube/table; resetting directly into the
    # pickup pose previously caused colliding/deflected robot geometry.
    home_pose=Pose(0.38,0,0.43 if args.scene=='grasp' else 0.35)
    initial=inverse_kinematics(home_pose)
    data.qpos[:4]=initial
    data.qpos[4:6]=0.025
    data.ctrl[:4]=initial
    data.ctrl[4:6]=0.025
    mujoco.mj_forward(model,data)
    ctrl=Controller()
    ctrl.reset_manual_target(height=home_pose.z)
    cmd=tuple(initial)
    src=DemoSource(pattern=args.demo_pattern) if args.mode=='demo' else (SerialSource(args.port) if args.mode=='serial' else ReplaySource(args.file))
    recording=Recorder(args.record) if args.record else None
    logger=SessionLog(args.log or str(ROOT/'logs'/'last_session.csv'))
    last_sample=None
    start=time.monotonic()
    last_report=start
    last_logger=start
    prev_state=None
    last_calibration_reported=0
    quit_flag=False
    park_mode='LIVE'  # LIVE, HOMING, PARKED; maintained by the app, not viewer key bindings
    pending_scene_reset=False
    terminal_commands=queue.SimpleQueue()
    # Independent terminal input avoids MuJoCo GUI shortcuts, especially Backspace and PageUp.
    if args.scene == "grasp" and not args.headless:
        def console_commands():
            while True:
                try: line=input()
                except (EOFError, OSError): return
                if line.strip().lower() in ("cube", "reset cube", "resetcube"):
                    terminal_commands.put("reset_cube")
        threading.Thread(target=console_commands, daemon=True).start()
    home_joints=inverse_kinematics(home_pose)
    finger_cmd=0.025  # Legacy standard-scene finger target.
    held_grip=HeldGrip()
    last_grip_time=time.monotonic()
    last_button_time=last_grip_time
    last_grip_action='HOLD'
    was_clamped=False

    def on_key(keycode):
        nonlocal quit_flag,park_mode,pending_scene_reset
        # Backspace safeguard: if native viewer reset wiped qpos/ctrl to 0, immediately restore arm joints to commanded target
        if args.scene == 'grasp' and keycode == 259:
            data.qpos[:4] = cmd
            data.qpos[4:6] = finger_command
            data.ctrl[:4] = cmd
            data.ctrl[4:6] = finger_command
            mujoco.mj_forward(model, data)
            print('[RESET WARNING] Backspace is MuJoCo full scene reset. Arm restored to target. Use DELETE for cube-only reset.')
            return
        # DELETE (GLFW keycode 261) or reset_cube: custom CUBE-ONLY reset.
        if args.scene == 'grasp' and (keycode == 261 or action_for_key(keycode) == 'reset_cube'):
            pending_scene_reset = True
            terminal_commands.put('reset_cube')
            print('[CUBE ONLY / DELETE] Queued cube-only reset; arm controls and target stay unchanged.')
            return
        # Grasp mode uses physical key HELD state; ignore press callbacks.
        # Enter is therefore not calibration here (use F11).
        if args.scene=='grasp' and keycode in (54,257,335):
            return
        # Digit controls and optional function-key / passive-viewer aliases.
        action=action_for_key(keycode)
        if action=='reset_cube':
            print('[CUBE RESET] Ignored in standard scene (no cube).')
            return
        if action=='estop':
            ctrl.emergency_stop()
            print('[SIM-STOP] Physics paused at current joint positions. Press SPACE (or Fn+F8) to reset.')
        elif action=='reset':
            ctrl.reset_emergency()
            park_mode='LIVE'
            print('[RESET] Software stop released. Live demo tracking resumes after calibration.')
        elif action=='calibrate':
            ctrl.calibrate_next()
            print('[CAL] Waiting for next valid sensor frame. In stationary demo, calibration will not visibly move the arm.')
        elif action=='gripper':
            ctrl.toggle_gripper()
            print('[GRIPPER]', 'OPEN' if ctrl.gripper_open else 'CLOSED')
        elif action in ('raise','lower','left','right','forward','backward'):
            if ctrl.estop or park_mode!='LIVE':
                print('[JOG] Ignored: release stop or exit parked mode before moving.')
            else:
                step=0.015  # 15 mm per key event
                if action=='raise': ctrl.set_height(step)
                elif action=='lower': ctrl.set_height(-step)
                elif action=='left': ctrl.nudge_xy(dy=step)
                elif action=='right': ctrl.nudge_xy(dy=-step)
                elif action=='forward': ctrl.nudge_xy(dx=step)
                elif action=='backward': ctrl.nudge_xy(dx=-step)
                print(f'[JOG] {action.upper()} | target=({ctrl.last_target.x:.3f}, '
                      f'{ctrl.last_target.y:.3f}, {ctrl.last_target.z:.3f}) m')
        elif action=='home':
            if ctrl.estop:
                print('[HOME] Reset with SPACE first; stop remains engaged.')
            else:
                park_mode='HOMING'
                ctrl.reset_manual_target(height=home_pose.z)
                print('[HOME] Returning to home pose; inputs ignored until END (resume live).')
        elif action=='live':
            if ctrl.estop:
                print('[LIVE] Reset stop with SPACE first.')
            else:
                park_mode='LIVE'
                ctrl.calibrate_next()
                print('[LIVE] Resuming real-time motion from current arm pose.')
        elif action=='quit':quit_flag=True

    def run_loop(viewer=None):
        nonlocal cmd,last_sample,last_report,last_logger,prev_state,park_mode,last_calibration_reported,pending_scene_reset,finger_cmd,last_grip_time,last_grip_action,was_clamped,last_button_time
        print('[ACTIVE PHANTOMARM CODE]', Path(__file__).resolve())
        print('\nPHANTOMARM v1.9 | Mode:',args.mode)
        print('ARROWS: UP/DOWN = height Z; LEFT/RIGHT = sideways Y (15 mm per press)')
        print('7/8 = forward/back X | 6 = gripper | 9 = STOP | SPACE = reset')
        print('HOME = park | END = resume | ENTER = calibrate | DELETE = reset cube')
        if args.scene=='grasp':
            print('CUBE RESET: Press DELETE in viewer, or type CUBE in terminal (NEVER Backspace).')
            print('HOLD 6 = tighten fingers | HOLD ENTER = loosen fingers | RELEASE = maintain grip')
            print('F11 = recalibrate (ENTER now opens fingers gradually)')
        else:
            print('BACKSPACE = alternative HOME shortcut')
        print('Optional function keys: F8 reset, F9 home, F10 live, F11 calibrate, F12 quit')
        if args.mode=='demo':
            print('[DEMO PATTERN]', args.demo_pattern,
                  '- fixed X/Y for controlled tests' if args.demo_pattern=='still' else '- animated two-axis IMU sweep')
        print('Avoid letter keys and 0-5: MuJoCo uses them for its own rendering and visibility.')
        if args.scene == 'grasp':
            print('[GRASP CHALLENGE] Orange CUBE is physical, orange square is just a goal marker.')
            print('Stable orange START platform: (x=.43,y=.00); green DROP platform: (x=.43,y=.12).')
            print('Use arrows to align fingers. HOLD 6 until pads press cube; release 6 to hold.')
            print('Lift slowly with UP. HOLD ENTER while placing to gradually release the cube.')
            print('If cube falls: move arm CLEAR of cube stand, press DELETE in viewer. NEVER Backspace.')
            print('Finger command holds position when no grip key is held; the platform supports cube at startup.')
        else:
            print('Orange marker = NON-PHYSICAL target site. Use run_grasp_demo for a real cube.')
        print('In serial mode IMU controls X/Y; keyboard nudges still apply. ENTER re-zeroes sensor tilt.')
        while not quit_flag and (viewer is None or viewer.is_running()):
            step_start=time.monotonic()
            # PHANTOMARM_INSERT_RESET_HOTKEY_V1
            if args.scene == 'grasp' and not args.headless and insert_just_pressed():
                terminal_commands.put('reset_cube')
                print('[CUBE ONLY / INSERT] Request queued; arm/controller unchanged.')
            if args.duration and step_start-start>=args.duration:break
            try: sample=src.poll(step_start)
            except Exception as exc:
                print('[SERIAL/REPLAY ERROR]',exc)
                ctrl.emergency_stop()
                break
            if sample is not None:
                last_sample=sample
                ctrl.consume(sample,step_start)
                if ctrl.calibration_count > last_calibration_reported:
                    last_calibration_reported = ctrl.calibration_count
                    cal = ctrl.last_calibration
                    print(f'[CAL OK] Neutral roll={cal[0]:+.2f}°, pitch={cal[1]:+.2f}° '
                          f'(capture #{ctrl.calibration_count}). In still mode, no visible change is expected.')
                if recording:recording.write(sample,step_start)
            while not terminal_commands.empty():
                command_request=terminal_commands.get_nowait()
                if command_request=='reset_cube':
                    pending_scene_reset=True
                    print('[CUBE RESET] Terminal request received (cube ONLY).')
            # Restart only when requested in the grasp scene. Source data and
            # hardware connections are preserved; viewer stays open.
            do_scene_reset = args.scene=='grasp' and pending_scene_reset
            if do_scene_reset:
                pending_scene_reset=False
                # Cube alone resets inside physics_update after an obstruction check.
                # Do NOT touch cmd, target, arm qpos, gripper or calibration.
            status=ctrl.tick(step_start)
            # PA2 height buttons are active only with fresh, valid serial data.
            # No movement in demo/replay, while stopped/parked, or on disconnect.
            pa2_active=(args.mode=='serial' and park_mode=='LIVE'
                        and status.state==State.READY
                        and last_sample is not None and last_sample.valid
                        and getattr(last_sample,'protocol','PA1')=='PA2'
                        and ctrl.last_seen is not None
                        and step_start-ctrl.last_seen<=ctrl.stale_after)
            dt_button=min(max(step_start-last_button_time,0.0),0.05)
            last_button_time=step_start
            if pa2_active and last_sample.z_up != last_sample.z_down:
                # 3.5 cm/s while held; reuse existing IK/workspace protection.
                direction=1.0 if last_sample.z_up else -1.0
                ctrl.set_height(direction*0.035*dt_button)
                status=ctrl.tick(step_start)  # refresh target after the height change
            effective_target=(clamp_grasp_target(status.target) if args.scene=='grasp' else status.target)
            clamped=(effective_target != status.target)
            if clamped:
                ctrl.height=effective_target.z
                ctrl.last_target=effective_target
                if not was_clamped:
                    print('[WORKSPACE GUARD] Pedestal top: target Z clamped at 0.350m. Move away to lower further.')
            was_clamped=clamped
            if status.state != prev_state:
                print(f'[{status.state.value}] {status.reason}')
                prev_state=status.state
            # The animated demo keeps producing new IMU samples. To avoid interrupting
            # a home move, HOMING and PARKED ignore the demo's live target updates.
            if park_mode=='HOMING' and not ctrl.estop:
                cmd=slew(cmd,home_joints,0.02)
                # Complete homing only when the *physical model* is near the target.
                # This avoids snapping from a halfway position to PARKED.
                if all(abs(float(data.qpos[i])-home_joints[i])<0.035 for i in range(4)):
                    if all(abs(cmd[i]-home_joints[i])<0.012 for i in range(4)):
                        park_mode='PARKED'
                        print('[HOME] Parked at home. Press END or F10 to resume.')
            elif park_mode=='LIVE' and status.state==State.READY:
                try:
                    goal=inverse_kinematics(effective_target)
                    # Slow motion while pinching a cube to reduce inertial slip.
                    step_size=(0.009 if args.scene=='grasp' else 0.02)
                    cmd=slew(cmd,goal,step_size)
                except UnreachableTarget as err:
                    pass
            # Keep the last accepted joint command while pausing. When paused,
            # MuJoCo is not stepped at all; gravity cannot move the virtual arm.
            # Limit finger closing speed, especially important for contact-based
            # grasping: an instantaneous position command may kick a cube away.
            if args.scene=='grasp':
                close_held,open_held=held_gripper_keys()
                if pa2_active:
                    # Combine physical buttons with existing keyboard held keys.
                    close_held=close_held or last_sample.grip_close
                    open_held=open_held or last_sample.grip_open
                if close_held and open_held:
                    # Opposing requests cancel rather than drive the gripper.
                    close_held=open_held=False
                dt_grip=min(max(step_start-last_grip_time, 0.0), 0.05)
                last_grip_time=step_start
                # E-stop locks the grip target; no unexpected opening.
                grip_value=held_grip.step(dt_grip,
                                          close_held and not ctrl.estop,
                                          open_held and not ctrl.estop)
                finger_command=(grip_value,grip_value)
                grip_action=('TIGHTEN' if close_held and not open_held else
                             'LOOSEN' if open_held and not close_held else 'HOLD')
                if grip_action!=last_grip_action:
                    print(f'[GRIP] {grip_action} | finger target={grip_value*1000:.1f} mm from closed')
                    last_grip_action=grip_action
            else:
                finger_target=0.025 if ctrl.gripper_open else 0.0
                finger_cmd+=max(-0.025,min(0.025,finger_target-finger_cmd))
                finger_command=(finger_cmd,finger_cmd)
            freeze = ctrl.estop or (park_mode=='LIVE' and ctrl.state!=State.READY) or park_mode=='PARKED'
            target_site=home_pose if park_mode!='LIVE' else effective_target
            def physics_update():
                nonlocal cmd
                if do_scene_reset:
                    if cube_spawn_is_clear(mujoco,model,data):
                        # No global mj_resetData: restore cube free-joint only.
                        # Resetting the cube must NOT change any robot joint or motor targets.
                        robot_q_before=data.qpos[:6].copy()
                        robot_v_before=data.qvel[:6].copy()
                        old_ctrl=data.ctrl[:6].copy()
                        restore_cube_only(mujoco,model,data)
                        assert all(abs(float(a)-float(b))<1e-12 for a,b in zip(data.qpos[:6],robot_q_before))
                        assert all(abs(float(a)-float(b))<1e-12 for a,b in zip(data.qvel[:6],robot_v_before))
                        assert all(abs(float(a)-float(b))<1e-12 for a,b in zip(data.ctrl[:6],old_ctrl))
                        print('[CUBE RESET OK] ONLY cube qpos/qvel restored; arm joint states, targets, and controller state unchanged.')
                    else:
                        print('[CUBE RESET BLOCKED] Arm intersects cube spawn; move arm aside, press Delete again.')
                data.ctrl[:4]=cmd
                data.ctrl[4:6]=finger_command
                # In grasp mode, display orange marker at the predicted GRASP
                # center, not at the wrist IK site (which is 5cm behind pads).
                if args.scene=='grasp':
                    heading=math.atan2(target_site.y,target_site.x)
                    model.site_pos[goal_id]=[
                        target_site.x+0.05*math.cos(heading),
                        target_site.y+0.05*math.sin(heading),
                        target_site.z+0.005]
                else:
                    model.site_pos[goal_id]=[target_site.x,target_site.y,target_site.z]
                if freeze:
                    mujoco.mj_forward(model,data)
                else:
                    for _ in range(4):mujoco.mj_step(model,data)
                return tuple(float(v) for v in data.site_xpos[tip_id])
            if viewer is not None:
                with viewer.lock():tip=physics_update()
                viewer.sync()
            else:
                tip=physics_update()
            error=math.dist(tip,(effective_target.x,effective_target.y,effective_target.z))
            if step_start-last_logger>=0.1:
                logger.row(step_start-start,args.mode,ctrl.state.value,last_sample,effective_target,tip,error)
                last_logger=step_start
            if step_start-last_report>=1.5:
                if args.scene=='grasp':
                    cube_pos = tuple(float(x) for x in data.xpos[cube_body_id])
                    gripping = sum(1 for i in range(data.ncon)
                                   if cube_geom_id in (data.contact[i].geom1, data.contact[i].geom2)
                                   and model.geom_bodyid[data.contact[i].geom1 if data.contact[i].geom2==cube_geom_id else data.contact[i].geom2]
                                   in (mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'left_finger'),
                                       mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'right_finger')))
                    hand_xyz=tuple(float(x) for x in data.site_xpos[grip_center_id])
                    print(f'[CUBE] xyz=({cube_pos[0]:.3f},{cube_pos[1]:.3f},{cube_pos[2]:.3f})m | hand=({hand_xyz[0]:.3f},{hand_xyz[1]:.3f},{hand_xyz[2]:.3f})m | finger contacts={gripping} | '+
                          ('LIFTED' if cube_pos[2]>0.395 else 'ON STAND / NOT LIFTED'))
                if args.scene=='grasp' and park_mode=='LIVE' and error>0.055:
                    print(f'[TRACKING WARNING] Tool is {error*100:.1f} cm from the ORANGE target. '
                          'The orange marker shows a request, not actual arm position; check collisions and allow movement to settle.')
                print(f'[t={step_start-start:5.1f}s] {ctrl.state.value} '
                      f'target=({effective_target.x:.3f},{effective_target.y:.3f},{effective_target.z:.3f}) '
                      f'joint-tip-error={error*100:.1f}cm')
                last_report=step_start
            time.sleep(max(0.0,0.02-(time.monotonic()-step_start)))
    try:
        if args.headless:run_loop()
        else:
            with mujoco.viewer.launch_passive(model,data,key_callback=on_key) as viewer:
                run_loop(viewer)
    finally:
        src.close()
        if recording:recording.close()
        logger.close()
        print('Session saved to',args.log or str(ROOT/'logs'/'last_session.csv'))
    return 0

if __name__=='__main__':raise SystemExit(main())
