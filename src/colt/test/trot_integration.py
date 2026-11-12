#!/usr/bin/env python3
"""Integration check against a running Colt Gazebo instance; sends stand/trot commands."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rospy
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import GetModelState
from legged_gazebo.msg import TorqueDiagnostics
from ocs2_msgs.msg import mpc_observation, mpc_target_trajectories
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=float, default=0.08)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--start-controller', action='store_true')
    parser.add_argument('--interrupt-after', type=float, help='send SIGINT after this many simulated seconds')
    parser.add_argument('--kill-after', type=float, help='drop the velocity publisher to test the command timeout')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rospy.init_node('colt_trot_integration', anonymous=True)
    lock = threading.Lock()
    data = {'ready': False, 'obs': None, 'peak_effort': 0.0, 'errors': [], 'modes': Counter(),
            'height_min': float('inf'), 'height_max': 0.0, 'tilt': 0.0,
            'feet_max': {leg: 0.0 for leg in ('LF', 'RF', 'LH', 'RH')}, 'record': False, 'target': None, 'torque': None}

    def observe(msg):
        with lock:
            data['obs'] = msg
            state, control = list(msg.state.value), list(msg.input.value)
            if len(state) != 24 or len(control) != 24 or not all(math.isfinite(v) for v in state + control):
                data['errors'].append('Invalid observation')
            elif data['record']:
                data['modes'][msg.mode] += 1
                data['height_min'] = min(data['height_min'], state[8])
                data['height_max'] = max(data['height_max'], state[8])
                data['tilt'] = max(data['tilt'], abs(state[10]), abs(state[11]))

    def effort(msg):
        with lock:
            if msg.effort:
                if not all(math.isfinite(v) for v in msg.effort):
                    data['errors'].append('Nonfinite joint effort')
                data['peak_effort'] = max(data['peak_effort'], max(abs(v) for v in msg.effort))
                if any(abs(v) > 20.0 + 1e-9 for v in msg.effort):
                    data['errors'].append('Measured joint effort exceeded 20 N m')

    def torque(msg):
        with lock:
            data['torque'] = msg
            values = list(msg.requested_effort) + list(msg.applied_effort) + list(msg.measured_effort)
            if not all(math.isfinite(v) for v in values):
                data['errors'].append('Nonfinite torque diagnostic')
            if any(v > 20.0 + 1e-9 for v in list(msg.peak_applied_effort) + list(msg.peak_measured_effort)):
                data['errors'].append('Full-rate torque audit exceeded 20 N m')

    def ready(msg):
        with lock:
            data['ready'] = msg.data

    def target(msg):
        with lock:
            data['target'] = msg

    def links(msg):
        with lock:
            if data['record']:
                for leg in data['feet_max']:
                    name = 'colt::' + leg + '_FOOT'
                    if name in msg.name:
                        data['feet_max'][leg] = max(data['feet_max'][leg], msg.pose[msg.name.index(name)].position.z)

    subscriptions = [rospy.Subscriber('/legged_robot_mpc_observation', mpc_observation, observe, queue_size=1000),
                     rospy.Subscriber('/joint_states', JointState, effort, queue_size=100),
                     rospy.Subscriber('/legged_robot_mpc_ready', Bool, ready, queue_size=1),
                     rospy.Subscriber('/gazebo/link_states', LinkStates, links, queue_size=10),
                     rospy.Subscriber('/torque_diagnostics', TorqueDiagnostics, torque, queue_size=10),
                     rospy.Subscriber('/legged_robot_mpc_target', mpc_target_trajectories, target, queue_size=1)]

    def wait_until(predicate, timeout=180.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with lock:
                assert not data['errors'], data['errors'][:3]
            if predicate():
                return
            time.sleep(0.02)
        raise AssertionError('Timed out waiting for simulator/controller')

    rospy.wait_for_service('/colt_gait/set_trot', timeout=180.0)
    gait = rospy.ServiceProxy('/colt_gait/set_trot', SetBool)
    rospy.wait_for_service('/controller_manager/list_controllers', timeout=180.0)
    listing = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
    model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    expected = 'controllers/legged_cheater_controller' if rospy.get_param('/colt_bringup/cheater') else 'controllers/legged_controller'
    wait_until(lambda: any(c.name == expected for c in listing().controller))
    if args.start_controller:
        assert not gait(True).success, 'Trot must be rejected before MPC/WBC is ready'
        settle_start = rospy.Time.now().to_sec()
        wait_until(lambda: rospy.Time.now().to_sec() - settle_start > 5.0)
        unpowered = model('colt', 'world')
        assert unpowered.success and unpowered.pose.position.z < 0.10, 'Expected unpowered crouch'
        switch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        request = SwitchControllerRequest(start_controllers=[expected], stop_controllers=[], strictness=2, start_asap=True)
        request.timeout = 5.0
        assert switch(request).ok, 'Controller start failed'
    def policy_ready():
        if any(c.name == expected and c.state == 'stopped' for c in listing().controller):
            raise AssertionError('Controller stopped before the initial policy was ready')
        return data['ready'] and data['obs'] is not None
    wait_until(policy_ready)
    initial = model('colt', 'world')
    assert initial.success
    q = initial.pose.orientation
    heading = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    with lock:
        data['record'] = True
    command = [sys.executable, '-B', str(Path(__file__).resolve().parents[1] / 'scripts/trot_forward.py'),
               '--speed', str(args.speed), '--duration', str(args.duration)]
    child = subprocess.Popen(command)
    start_time = rospy.Time.now().to_sec()
    interrupted = False
    killed = False
    try:
        deadline = time.monotonic() + 600.0
        while child.poll() is None and time.monotonic() < deadline:
            if args.interrupt_after is not None and not interrupted and rospy.Time.now().to_sec() - start_time > args.interrupt_after:
                child.send_signal(signal.SIGINT)
                interrupted = True
            if args.kill_after is not None and not killed and rospy.Time.now().to_sec() - start_time > args.kill_after:
                child.kill()
                killed = True
            time.sleep(0.05)
        if child.poll() is None:
            raise AssertionError('Forward trot script did not finish within 600 wall seconds')
        assert child.returncode == (-signal.SIGKILL if killed else 130 if interrupted else 0), 'Forward trot script failed: ' + str(child.returncode)
        if killed:
            loss_time = rospy.Time.now().to_sec()
            wait_until(lambda: rospy.Time.now().to_sec() - loss_time > 3.0)
            with lock:
                stopped_target = data['target']
            assert stopped_target is not None and all(abs(v) < 1e-9 for v in stopped_target.stateTrajectory[-1].value[:3]), 'Velocity timeout did not request zero momentum'
            assert gait(False).success
            stance_time = rospy.Time.now().to_sec()
            wait_until(lambda: rospy.Time.now().to_sec() - stance_time > 4.0)
    finally:
        if child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
    final = model('colt', 'world')
    dx = final.pose.position.x - initial.pose.position.x
    dy = final.pose.position.y - initial.pose.position.y
    forward = dx * math.cos(heading) + dy * math.sin(heading)
    lateral = -dx * math.sin(heading) + dy * math.cos(heading)
    running = {c.name for c in listing().controller if c.state == 'running'}
    with lock:
        obs = data['obs']
        audit = data['torque']
        assert audit is not None and len(audit.name) == 12, 'Full-rate torque audit is required'
        torque_peaks = [dict(joint=name, requested=audit.peak_requested_effort[i], applied=audit.peak_applied_effort[i],
                            measured=audit.peak_measured_effort[i], saturations=audit.saturation_count[i],
                            time=audit.peak_time[i], phase=audit.peak_phase[i], p=audit.peak_position_term[i],
                            d=audit.peak_velocity_term[i], ff=audit.peak_feedforward[i], dq=audit.peak_measured_velocity[i])
                       for i, name in enumerate(audit.name)]
        result = dict(speed=args.speed, duration=args.duration, interrupted=interrupted, publisher_killed=killed, forward_m=forward, lateral_m=lateral,
                      final_actual_height_m=final.pose.position.z, final_estimated_height_m=obs.state.value[8],
                      final_actual_speed_mps=math.hypot(final.twist.linear.x, final.twist.linear.y), final_mode=obs.mode,
                      height_range_m=[data['height_min'], data['height_max']], max_tilt_rad=data['tilt'],
                      peak_effort_Nm=data['peak_effort'], full_rate_samples=audit.samples, torque_peaks=torque_peaks,
                      contact_modes=dict(data['modes']), feet_max_center_height_m=data['feet_max'])
        assert not data['errors'], data['errors'][:3]
    print(json.dumps(result, indent=2), flush=True)
    if args.output:
        args.output.write_text(json.dumps(result, indent=2) + '\n')
    assert final.success and expected in running and 'controllers/joint_state_controller' in running
    assert result['contact_modes'].get(9, 0) > 20 and result['contact_modes'].get(6, 0) > 20, 'Both diagonal support pairs must occur'
    assert all(h > 0.035 for h in result['feet_max_center_height_m'].values()), 'All four feet must leave the ground'
    assert result['peak_effort_Nm'] <= 20.0 + 1e-9 and result['max_tilt_rad'] < 0.20
    assert 0.24 < result['height_range_m'][0] <= result['height_range_m'][1] < 0.33
    assert result['final_mode'] == 15 and 0.27 < result['final_actual_height_m'] < 0.30
    assert result['final_actual_speed_mps'] < 0.03, 'Robot must stop after the demo'
    if not interrupted and not killed:
        assert forward > 0.5 * args.speed * args.duration, 'Insufficient actual forward displacement'
        assert abs(lateral) < 0.15, 'Excessive lateral drift'
    print('PASS: trot forward, diagonal support, foot clearance, torque, and return to stance', flush=True)


if __name__ == '__main__':
    main()
