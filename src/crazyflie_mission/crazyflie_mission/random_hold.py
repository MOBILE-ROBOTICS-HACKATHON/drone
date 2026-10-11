#!/usr/bin/env python3
"""
Random hold mission for a Crazyflie with Lighthouse positioning (Crazyswarm2).

Sequence: wait for a live pose, take off, fly to ONE random point inside a
safe box, hold there, and keep broadcasting the drone's position throughout.

Topics published:
  /<cf>/position        geometry_msgs/PointStamped  live drone position
  /<cf>/target          geometry_msgs/PointStamped  the chosen hold point
  /<cf>/mission_status  std_msgs/String             current state
"""

import math
import random
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import String
from crazyflie_interfaces.srv import Takeoff, GoTo, Land, Arm



def to_duration(seconds: float) -> Duration:
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int((seconds - sec) * 1e9))


class State(Enum):
    WAIT_FOR_POSE = 'WAIT_FOR_POSE'
    TAKING_OFF = 'TAKING_OFF'
    FLYING_TO_TARGET = 'FLYING_TO_TARGET'
    HOLDING = 'HOLDING'
    LANDING = 'LANDING'
    DONE = 'DONE'


class RandomHold(Node):
    def __init__(self):
        super().__init__('random_hold')
        p = self.declare_parameter
        self.cf = p('cf_name', 'cf231').value
        self.takeoff_height = p('takeoff_height', 0.5).value
        self.takeoff_time = p('takeoff_time', 2.5).value
        self.speed = p('speed', 0.3).value            # m/s, sets go_to duration
        self.hold_time = p('hold_time', 30.0).value    # s; 0 or less = hold until Ctrl+C
        self.pose_timeout = p('pose_timeout', 1.0).value  # s without pose => land
        self.box = {
            'x': (p('x_min', -0.5).value, p('x_max', 0.5).value),
            'y': (p('y_min', -0.5).value, p('y_max', 0.5).value),
            'z': (p('z_min', 0.4).value, p('z_max', 1.0).value),
        }
        seed = p('seed', -1).value
        if seed >= 0:
            random.seed(seed)

        self.state = State.WAIT_FOR_POSE
        self.pose = None
        self.last_pose_t = None
        self.target = None
        self.phase_end = 0.0

        ns = f'/{self.cf}'
        self.create_subscription(PoseStamped, f'{ns}/pose', self.on_pose, 10)
        self.pos_pub = self.create_publisher(PointStamped, f'{ns}/position', 10)
        self.target_pub = self.create_publisher(PointStamped, f'{ns}/target', 10)
        self.status_pub = self.create_publisher(String, f'{ns}/mission_status', 10)

        self.arm_cli = self.create_client(Arm, f'{ns}/arm')
        self.armed_at = None


        self.takeoff_cli = self.create_client(Takeoff, f'{ns}/takeoff')
        self.goto_cli = self.create_client(GoTo, f'{ns}/go_to')
        self.land_cli = self.create_client(Land, f'{ns}/land')

        self.timer = self.create_timer(0.1, self.step)
        self.get_logger().info(f'Random hold mission ready for {self.cf}, box {self.box}')

    # helpers
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def xyz(self):
        p = self.pose.pose.position
        return p.x, p.y, p.z

    def set_state(self, s: State):
        self.state = s
        self.get_logger().info(f'State -> {s.value}')

    # callbacks
    def on_pose(self, msg: PoseStamped):
        self.pose = msg
        self.last_pose_t = self.now()
        out = PointStamped()
        out.header = msg.header
        out.point = msg.pose.position
        self.pos_pub.publish(out)
        x, y, z = self.xyz()
        self.get_logger().info(
            f'[{self.state.value}] position x={x:+.3f} y={y:+.3f} z={z:+.3f} m',
            throttle_duration_sec=1.0)

    def step(self):
        self.status_pub.publish(String(data=self.state.value))
        t = self.now()

        # Safety: if position stream stops while airborne, land.
        airborne = self.state in (State.TAKING_OFF, State.FLYING_TO_TARGET, State.HOLDING)
        if airborne and (self.last_pose_t is None or t - self.last_pose_t > self.pose_timeout):
            self.get_logger().error('Lost position stream, landing now')
            self.land()
            return

        if self.state == State.WAIT_FOR_POSE:
            clients = (self.takeoff_cli, self.goto_cli, self.land_cli, self.arm_cli)
            if self.pose is None or not all(c.service_is_ready() for c in clients):
                self.get_logger().info('Waiting for pose and Crazyswarm2 services...',
                                       throttle_duration_sec=2.0)
                return
            x, y, z = self.xyz()
            if z > 0.15:
                print("Height issue")
                self.get_logger().warn(f'Start height z={z:.2f} m looks wrong, check the '
                                       'Lighthouse lock before flying', throttle_duration_sec=2.0)
                return
            if self.armed_at is None:
                self.arm_cli.call_async(Arm.Request(arm=True))
                self.armed_at = t
                self.get_logger().info('Arm request sent')
                return
            if t - self.armed_at < 1.0:   # give the supervisor a moment
                return
            req = Takeoff.Request()
            req.group_mask = 0
            req.height = float(self.takeoff_height)
            req.duration = to_duration(self.takeoff_time)
            self.takeoff_cli.call_async(req)
            self.phase_end = t + self.takeoff_time + 0.5
            self.set_state(State.TAKING_OFF)

        elif self.state == State.TAKING_OFF and t >= self.phase_end:
            self.target = tuple(random.uniform(*self.box[a]) for a in ('x', 'y', 'z'))
            dist = math.dist(self.xyz(), self.target)
            dur = max(2.0, dist / self.speed)
            req = GoTo.Request()
            req.group_mask = 0
            req.relative = False
            req.goal = Point(x=self.target[0], y=self.target[1], z=self.target[2])
            req.yaw = 0.0
            req.duration = to_duration(dur)
            self.goto_cli.call_async(req)
            self.phase_end = t + dur + 0.5
            self.get_logger().info(
                'Random target x={:+.3f} y={:+.3f} z={:+.3f} m, {:.1f} s flight'.format(
                    *self.target, dur))
            self.set_state(State.FLYING_TO_TARGET)

        elif self.state == State.FLYING_TO_TARGET and t >= self.phase_end:
            self.phase_end = t + self.hold_time
            self.set_state(State.HOLDING)

        elif self.state == State.HOLDING:
            err = math.dist(self.xyz(), self.target)
            self.get_logger().info(f'Holding, error to target {err*100:.1f} cm',
                                   throttle_duration_sec=2.0)
            if self.hold_time > 0 and t >= self.phase_end:
                self.land()

        elif self.state == State.LANDING and t >= self.phase_end:
            self.set_state(State.DONE)

        if self.target is not None:
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.point = Point(x=self.target[0], y=self.target[1], z=self.target[2])
            self.target_pub.publish(msg)

    def land(self):
        z = self.xyz()[2] if self.pose else self.takeoff_height
        dur = max(2.0, z / 0.25)
        req = Land.Request()
        req.group_mask = 0
        req.height = 0.0
        req.duration = to_duration(dur)
        fut = self.land_cli.call_async(req)
        self.phase_end = self.now() + dur + 0.5
        self.set_state(State.LANDING)
        return fut


def main():
    # Own the Ctrl+C handling so we can still send a land command after it.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RandomHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.state in (State.TAKING_OFF, State.FLYING_TO_TARGET, State.HOLDING):
            node.get_logger().warn('Ctrl+C received, landing')
            fut = node.land()
            rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
