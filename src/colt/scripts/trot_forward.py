#!/usr/bin/env python3
"""Run a bounded forward trot after the existing manual stand-up has completed."""
import argparse
import math
import signal
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import mpc_observation
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class ForwardTrot:
    def __init__(self):
        self.lock = threading.Lock()
        self.observation = None
        self.received_at = 0.0
        self.ready = False
        self.cancelled = False
        self.speed = 0.0
        self.command = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.observation_sub = rospy.Subscriber('/legged_robot_mpc_observation', mpc_observation, self.observe, queue_size=1)
        self.ready_sub = rospy.Subscriber('/legged_robot_mpc_ready', Bool, self.on_ready, queue_size=1)
        self.set_trot = rospy.ServiceProxy('/colt_gait/set_trot', SetBool)

    def observe(self, message):
        with self.lock:
            self.observation = message
            self.received_at = time.monotonic()

    def on_ready(self, message):
        with self.lock:
            self.ready = message.data

    def snapshot(self):
        with self.lock:
            return self.observation, self.received_at, self.ready

    def send(self, speed):
        command = Twist()
        command.linear.x = speed
        self.command.publish(command)
        self.speed = speed

    def switch_gait(self, trot):
        result = self.set_trot(trot)
        if not result.success:
            raise RuntimeError(result.message)
        rospy.loginfo(result.message)

    def check(self):
        observation, received_at, ready = self.snapshot()
        if not ready or observation is None:
            raise RuntimeError('MPC/WBC stopped or is not ready')
        if time.monotonic() - received_at > 5.0:
            raise RuntimeError('No fresh MPC observation for 5 wall seconds')
        state, control = list(observation.state.value), list(observation.input.value)
        if len(state) != 24 or len(control) != 24 or not all(math.isfinite(x) for x in state + control):
            raise RuntimeError('Invalid MPC observation')
        if not 0.18 < state[8] < 0.40 or max(abs(state[10]), abs(state[11])) > 0.35:
            raise RuntimeError('Body height or tilt exceeded the forward-trot demo limits')
        return observation

    def wait_for_standing(self, timeout):
        rospy.wait_for_service('/colt_gait/set_trot', timeout=timeout)
        deadline = time.monotonic() + timeout
        stable_since = None
        rospy.loginfo('Waiting for MPC/WBC and 2 simulated seconds of stable stance; use the existing manual stand-up command.')
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.cancelled:
                raise RuntimeError('Cancelled before walking started')
            observation, received_at, ready = self.snapshot()
            stable = False
            if ready and observation is not None and time.monotonic() - received_at < 2.0:
                observation = self.check()
                state = observation.state.value
                stable = (self.command.get_num_connections() > 0 and observation.mode == 15 and
                          0.27 < state[8] < 0.30 and max(abs(state[10]), abs(state[11])) < 0.05)
            if stable:
                if stable_since is None:
                    stable_since = observation.time
                if observation.time - stable_since >= 2.0:
                    return observation
            else:
                stable_since = None
            time.sleep(0.02)
        raise RuntimeError('Timed out waiting for stable stance')

    def segment(self, duration, velocity, allow_cancel=True):
        start = rospy.Time.now().to_sec()
        deadline = time.monotonic() + max(20.0, duration * 20.0)
        next_publish = start
        while not rospy.is_shutdown():
            if allow_cancel and self.cancelled:
                return False
            now = rospy.Time.now().to_sec()
            elapsed = now - start
            if elapsed < 0 or time.monotonic() > deadline:
                raise RuntimeError('Simulation time reset or stopped advancing')
            self.check()
            if now >= next_publish:
                self.send(velocity(min(elapsed, duration)))
                next_publish = now + 0.05
            if elapsed >= duration:
                return True
            time.sleep(0.005)
        raise RuntimeError('ROS shut down during trot')

    def stop(self, ramp):
        current_speed = self.speed
        rospy.loginfo('Slowing down, then returning to four-foot stance ...')
        self.segment(ramp, lambda t: current_speed * (1.0 - t / ramp), allow_cancel=False)
        self.segment(2.0, lambda _: 0.0, allow_cancel=False)
        self.switch_gait(False)
        self.segment(3.0, lambda _: 0.0, allow_cancel=False)


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be a finite positive number')
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=positive, default=0.08, help='forward speed in m/s (maximum 0.15)')
    parser.add_argument('--duration', type=positive, default=10.0, help='constant-speed duration in simulated seconds')
    parser.add_argument('--ramp', type=positive, default=2.0, help='acceleration/deceleration duration, at least 1 simulated second')
    parser.add_argument('--timeout', type=positive, default=120.0, help='wall-time limit for waiting for stable stance')
    args = parser.parse_args(rospy.myargv()[1:])
    if args.speed > 0.15 or args.ramp < 1.0:
        parser.error('speed must be <= 0.15 m/s and ramp must be >= 1 second')
    rospy.init_node('colt_forward_trot', anonymous=True, disable_signals=True)
    runner = ForwardTrot()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: setattr(runner, 'cancelled', True))
    started = False
    try:
        initial = runner.wait_for_standing(args.timeout)
        started = True
        runner.switch_gait(True)
        # GaitReceiver applies the template beyond the current 1 s MPC horizon.
        running = runner.segment(2.0, lambda _: 0.0)
        if running:
            rospy.loginfo('Forward trot: %.3f m/s, %.1f s plateau, %.1f s ramps', args.speed, args.duration, args.ramp)
            running = runner.segment(args.ramp, lambda t: args.speed * t / args.ramp)
        if running:
            runner.segment(args.duration, lambda _: args.speed)
        runner.stop(args.ramp)
        final = runner.check()
        state = final.state.value
        if (final.mode != 15 or not 0.27 < state[8] < 0.30 or max(abs(state[10]), abs(state[11])) > 0.08 or
                math.sqrt(sum(v * v for v in state[:3])) > 0.03):
            raise RuntimeError('Robot has not settled into stationary four-foot stance')
        dx = final.state.value[6] - initial.state.value[6]
        dy = final.state.value[7] - initial.state.value[7]
        yaw = initial.state.value[9]
        rospy.loginfo('Returned to stance. Estimated forward displacement: %.3f m', dx * math.cos(yaw) + dy * math.sin(yaw))
        return 130 if runner.cancelled else 0
    except (RuntimeError, rospy.ROSException, rospy.ServiceException) as error:
        rospy.logerr('Forward trot failed: %s', error)
        if started and not rospy.is_shutdown():
            runner.send(0.0)
            try:
                runner.switch_gait(False)
            except (RuntimeError, rospy.ROSException, rospy.ServiceException) as stop_error:
                rospy.logerr('Could not request stance: %s', stop_error)
        return 1
    finally:
        if started and not rospy.is_shutdown():
            runner.send(0.0)


if __name__ == '__main__':
    raise SystemExit(main())
