#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import time
import math

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # ---------- State Machine ----------
        self.state = 'READY'        # READY → WAITING → TURNING → MOVING_FORWARD → READY
        self.pending_command = None
        self.command_received_time = 0.0
        self.turn_start_time = 0.0
        self.move_start_time = 0.0

        # ---------- Distance ----------
        self.current_distance = 999.0
        self.last_distance_log = 999.0

        # ---------- Parameters ----------
        self.maneuver_delay = 3.0      # Wait 3 seconds after detection
        self.turn_duration = 1.05      # Turn for 1.05 seconds (≈94.5 degrees)
        self.turn_speed = 1.5          # Angular speed in rad/s
        self.move_duration = 2.0       # Move forward for 2 seconds after turn
        
        self.normal_speed = 0.3
        self.waiting_speed = 0.1
        self.turn_forward_speed = 0.1  # Small forward speed during turn
        self.after_turn_speed = 0.3    # Speed after completing turn
        
        self.safe_distance = 0.5

        # ---------- Subscribers ----------
        self.create_subscription(String, '/command', self.command_callback, 10)
        self.create_subscription(Float32, '/distance/front', self.distance_callback, 10)

        # ---------- Publisher ----------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---------- Timer ----------
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        self.get_logger().info("🚗 Controller started | State = READY")
        self.get_logger().info("📏 Safe distance: 0.5m")
        self.get_logger().info("🔄 Turn: 1.5 rad/s for 1.05s (≈94°)")

    # =====================================================
    def command_callback(self, msg):
        cmd = msg.data.strip().upper()  
        
        # 🛑 STOP command - always accepted
        if cmd == 'STOP':
            self.get_logger().warn("🛑 STOP received")
            self.state = 'STOPPING'
            self.pending_command = None
            return

        # Ignore if not in READY state
        if self.state != 'READY':
            self.get_logger().debug(f"Ignoring {cmd} (state: {self.state})")
            return

        # Accept only LEFT or RIGHT commands
        if cmd in ['LEFT', 'RIGHT']:
            self.pending_command = cmd
            self.command_received_time = time.time()
            self.state = 'WAITING'
            self.get_logger().info(f"🪧 {cmd} detected → WAITING for {self.maneuver_delay}s")
        else:
            self.get_logger().warn(f"Unknown command: {cmd}")

    # =====================================================
    def distance_callback(self, msg):
        self.current_distance = msg.data

        # Log significant distance changes
        if abs(msg.data - self.last_distance_log) > 0.5:
            self.get_logger().info(f"📏 Distance: {msg.data:.2f} m")
            self.last_distance_log = msg.data

        # Emergency stop if too close to obstacle
        if msg.data < 0.3 and self.state != 'STOPPING':
            self.get_logger().warn("⚠️ EMERGENCY STOP - Obstacle too close!")
            self.state = 'STOPPING'

    # =====================================================
    def timer_callback(self):
        twist = Twist()
        current_time = time.time()

        # ---------- Emergency stop check ----------
        if self.current_distance < 0.3:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            if self.state != 'STOPPING':
                self.state = 'STOPPING'
                self.get_logger().warn("⚠️ Emergency stop activated")
            return

        # ---------- READY state ----------
        if self.state == 'READY':
            if self.current_distance < self.safe_distance:
                # Too close to obstacle, stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().debug("Too close to obstacle, stopping")
            else:
                # Move forward normally
                twist.linear.x = self.normal_speed
                twist.angular.z = 0.0

        # ---------- WAITING state ----------
        elif self.state == 'WAITING':
            elapsed = current_time - self.command_received_time
            
            if elapsed < self.maneuver_delay:
                # Move slowly forward while waiting
                twist.linear.x = self.waiting_speed
                twist.angular.z = 0.0
                
                # Log remaining time every second
                if int(elapsed) != int(elapsed - 0.1):
                    remaining = self.maneuver_delay - elapsed
                    self.get_logger().debug(f"⏳ Waiting: {remaining:.1f}s remaining")
            else:
                # Wait period finished, start turning
                self.state = 'TURNING'
                self.turn_start_time = current_time
                self.get_logger().info(f"🔁 Starting {self.pending_command} turn")
                return  # Skip publishing this cycle

        # ---------- TURNING state ----------
        elif self.state == 'TURNING':
            elapsed = current_time - self.turn_start_time
            
            if elapsed < self.turn_duration:
                # Perform the turn
                twist.linear.x = self.turn_forward_speed  
                
                if self.pending_command == 'LEFT':
                    twist.angular.z = self.turn_speed  # Positive = counter-clockwise = LEFT
                elif self.pending_command == 'RIGHT':
                    twist.angular.z = -self.turn_speed  # Negative = clockwise = RIGHT
                else:
                    twist.angular.z = 0.0
                
                # Log remaining turn time
                if int(elapsed * 2) != int((elapsed - 0.1) * 2):  # Every 0.5 seconds
                    remaining = self.turn_duration - elapsed
                    self.get_logger().debug(f"🔄 Turning: {remaining:.1f}s remaining")
            else:
                # Turn completed, start moving forward
                self.state = 'MOVING_FORWARD'
                self.move_start_time = current_time
                self.get_logger().info("✅ Turn completed → Moving forward")
                return  # Skip publishing this cycle

        # ---------- MOVING_FORWARD state ----------
        elif self.state == 'MOVING_FORWARD':
            elapsed = current_time - self.move_start_time
            
            if elapsed < self.move_duration:
                # Move forward after turn
                twist.linear.x = self.after_turn_speed
                twist.angular.z = 0.0
                
                # Log remaining movement time
                if int(elapsed) != int(elapsed - 0.1):
                    remaining = self.move_duration - elapsed
                    self.get_logger().debug(f"🚀 Moving forward: {remaining:.1f}s remaining")
            else:
                # Movement completed, return to READY
                self.state = 'READY'
                self.pending_command = None
                self.get_logger().info("➡️ Maneuver complete → READY")
                return  # Skip publishing this cycle

        # ---------- STOPPING state ----------
        elif self.state == 'STOPPING':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
            # Stay in STOPPING for 3 seconds, then return to READY
            if current_time - self.command_received_time > 3.0:
                self.state = 'READY'
                self.pending_command = None
                self.get_logger().info("➡️ Back to READY after stop")

        # Publish the twist command
        self.cmd_pub.publish(twist)
        
        # State summary every 5 seconds
        if not hasattr(self, 'last_summary'):
            self.last_summary = current_time
        
        if current_time - self.last_summary > 5.0:
            self.get_logger().info(
                f"📊 State: {self.state} | "
                f"Cmd: {self.pending_command} | "
                f"Dist: {self.current_distance:.2f}m"
            )
            self.last_summary = current_time

    # =====================================================
    def destroy_node(self):
        """Clean shutdown"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info("🛑 Controller shutting down")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
