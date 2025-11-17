#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.clock import Clock
from rclpy.duration import Duration

class SquareRover(Node):
    def __init__(self):
        super().__init__('square_rover')

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.loop)

        # State machine
        self.state = "FORWARD"
        self.start = None
        self.side = 0
        self.clock = Clock()

        # Motion parameters (tune these for your robot)
        self.forward_speed = 0.25   # m/s
        self.forward_time = 5.0    # seconds (distance ≈ forward_speed * forward_time)
        self.turn_speed = 5.0      # rad/s
        # time to turn 90 degrees (pi/2) at turn_speed
        self.turn_time = 9.0
        self.get_logger().info(f"Square rover started. forward_time={self.forward_time}s, turn_time={self.turn_time:.2f}s")

    def elapsed(self, t):
        if self.start is None:
            return False
        return (self.clock.now() - self.start) > Duration(seconds=t)

    def cmd(self, x=0.0, z=0.0):
        # ensure floats
        msg = Twist()
        msg.linear.x = float(x)
        msg.angular.z = float(z)
        self.pub.publish(msg)

    def loop(self):
        # Safety: if something left start as None, initialize when entering state
        if self.state == "FORWARD":
            if self.start is None:
                self.start = self.clock.now()
                self.get_logger().info(f"Starting forward side {self.side+1}")

            if not self.elapsed(self.forward_time):
                self.cmd(self.forward_speed, 0.0)
            else:
                self.cmd(0.0, 0.0)
                self.start = None
                self.state = "TURN"

        elif self.state == "TURN":
            if self.start is None:
                self.start = self.clock.now()
                self.get_logger().info(f"Turning (side {self.side+1})")

            if not self.elapsed(self.turn_time):
                self.cmd(0.0, self.turn_speed)
            else:
                self.cmd(0.0, 0.0)
                self.start = None
                self.side += 1
                if self.side >= 4:
                    self.state = "DONE"
                else:
                    self.state = "FORWARD"

        elif self.state == "DONE":
            # stop and cancel timer to end node activity gracefully
            self.cmd(0.0, 0.0)
            self.get_logger().info("Square completed. Stopping timer.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = SquareRover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

