#!/usr/bin/env python3
"""Humble port of the MADER perfect tracker's state path for pair-01."""
# Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory,
# Massachusetts Institute of Technology. See the upstream MADER LICENSE.

import argparse
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from mader_msgs.msg import WhoPlans
from snapstack_msgs.msg import Goal, State


class PerfectTracker(Node):
    def __init__(self, namespace, start, yaw):
        super().__init__("perfect_tracker", namespace=namespace)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(State, "state", latched)
        self.authority_pub = self.create_publisher(WhoPlans, "who_plans", latched)
        self.create_subscription(Goal, "goal", self.follow_reference, 10)
        self.state = State()
        self.state.header.frame_id = "world"
        self.state.pos.x, self.state.pos.y, self.state.pos.z = start
        self.state.quat.z = math.sin(yaw / 2)
        self.state.quat.w = math.cos(yaw / 2)
        self.initial_timer = self.create_timer(1.0, self.publish_initial)

    def publish_initial(self):
        self.initial_timer.cancel()
        self.state_pub.publish(self.state)
        # Mirror the upstream commands node's simulated takeoff handoff.
        self.authority_timer = self.create_timer(0.1, self.publish_authority)

    def publish_authority(self):
        self.authority_timer.cancel()
        msg = WhoPlans()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.value = WhoPlans.MADER
        self.authority_pub.publish(msg)

    def follow_reference(self, goal):
        # Hopf-fibration attitude from upstream perfect_controller.py.
        thrust = (goal.a.x, goal.a.y, goal.a.z + 9.81)
        length = math.sqrt(sum(axis * axis for axis in thrust))
        a, b, c = (axis / length for axis in thrust)
        scale = 1.0 / math.sqrt(2 * (1 + c))
        x, y, w = -b * scale, a * scale, (1 + c) * scale
        half_yaw = goal.psi / 2
        sine, cosine = math.sin(half_yaw), math.cos(half_yaw)
        self.state.pos = goal.p
        self.state.vel = goal.v
        self.state.quat.x = x * cosine + y * sine
        self.state.quat.y = y * cosine - x * sine
        self.state.quat.z = w * sine
        self.state.quat.w = w * cosine
        self.state_pub.publish(self.state)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("namespace")
    parser.add_argument("start", nargs=3, type=float)
    parser.add_argument("yaw", type=float)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = PerfectTracker(args.namespace, args.start, args.yaw)
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()