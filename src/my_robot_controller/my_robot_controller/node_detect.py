#!/usr/bin/env python3
"""
Modified sign detector with corrected direction logic
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for commands
        self.publisher = self.create_publisher(String, '/command', 10)
        
        # Display windows
        cv2.namedWindow('Sign Detection', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Masks', cv2.WINDOW_NORMAL)
        
        # Color ranges (HSV)
        self.blue_lower = np.array([100, 100, 20])
        self.blue_upper = np.array([125, 255, 100])
        self.white_lower = np.array([0, 0, 80])
        self.white_upper = np.array([180, 30, 255])
        self.red_lower1 = np.array([0, 70, 50])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 70, 50])
        self.red_upper2 = np.array([180, 255, 255])
        
        # Confidence threshold
        self.confidence_threshold = 0.6
        
        # Anti-spam timer
        self.last_command_time = 0
        self.command_cooldown = 2.0  # seconds
        
        self.get_logger().info("✅ Sign Detector Started")
        
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            frame = cv2.resize(frame, (320, 240))
            debug_frame = frame.copy()
            
            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Create masks
            mask_blue = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
            mask_white = cv2.inRange(hsv, self.white_lower, self.white_upper)
            
            mask_red1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
            mask_red2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            
            # Morphological operations
            kernel = np.ones((5, 5), np.uint8)
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            blue_contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            current_time = time.time()
            can_send = (current_time - self.last_command_time) > self.command_cooldown
            
            # Process STOP signs first
            for cnt in red_contours:
                area = cv2.contourArea(cnt)
                if area > 400:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    
                    if can_send:
                        self.publisher.publish(String(data='STOP'))
                        self.last_command_time = current_time
                        self.get_logger().info("🛑 STOP detected")
                        cv2.putText(debug_frame, 'STOP', (x, y-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    return
            
            # Process direction signs
            best_confidence = 0
            best_direction = None
            best_contour = None
            
            for cnt in blue_contours:
                area = cv2.contourArea(cnt)
                if area < 300:
                    continue
                
                x, y, w, h = cv2.boundingRect(cnt)
                sign_center_x = x + w // 2
                
                # Extract white arrow region
                roi_white = mask_white[y:y+h, x:x+w]
                arrow_contours, _ = cv2.findContours(roi_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for arrow_cnt in arrow_contours:
                    if cv2.contourArea(arrow_cnt) < 80:
                        continue
                    
                    # Convert to global coordinates
                    arrow_global = arrow_cnt + np.array([[x, y]])
                    
                    # Find extreme points
                    left_x = np.min(arrow_global[:, :, 0])
                    right_x = np.max(arrow_global[:, :, 0])
                    
                    # Calculate distances
                    dl = abs(left_x - sign_center_x)
                    dr = abs(right_x - sign_center_x)
                    total = dl + dr
                    
                    if total == 0:
                        continue
                    
                    # ✅✅✅ CORRECTED LOGIC HERE ✅✅✅
                    if dl > dr:
                        # Left point is farther → arrow tip is on left → arrow points RIGHT
                        direction = 'RIGHT'
                        confidence = dl / total
                        color = (0, 255, 0)  # Green for RIGHT
                    else:
                        # Right point is farther → arrow tip is on right → arrow points LEFT
                        direction = 'LEFT'
                        confidence = dr / total
                        color = (0, 0, 255)  # Red for LEFT
                    
                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_direction = direction
                        best_contour = cnt
                    
                    # Draw contour
                    cv2.drawContours(debug_frame, [arrow_global], -1, color, 2)
            
            # Draw and publish best detection
            if best_contour is not None:
                x, y, w, h = cv2.boundingRect(best_contour)
                cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (255, 255, 0), 2)
                
                # Draw center line
                center_x = x + w // 2
                cv2.line(debug_frame, (center_x, y), (center_x, y + h), (255, 0, 255), 1)
                
                # Draw direction text with confidence
                text = f"{best_direction} ({best_confidence:.1f})"
                cv2.putText(debug_frame, text, (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                if best_confidence >= self.confidence_threshold and can_send:
                    self.publisher.publish(String(data=best_direction))
                    self.last_command_time = current_time
                    self.get_logger().info(f"📤 {best_direction} (conf: {best_confidence:.2f})")
            
            # Display masks
            masks_display = cv2.vconcat([
                cv2.cvtColor(mask_blue, cv2.COLOR_GRAY2BGR),
                cv2.cvtColor(mask_white, cv2.COLOR_GRAY2BGR),
                cv2.cvtColor(mask_red, cv2.COLOR_GRAY2BGR)
            ])
            
            cv2.imshow('Sign Detection', cv2.resize(debug_frame, (640, 480)))
            cv2.imshow('Masks', masks_display)
            
            if cv2.waitKey(1) == 27:  # ESC key
                raise KeyboardInterrupt
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
