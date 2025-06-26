#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
import torch
import message_filters
from image_geometry import PinholeCameraModel
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from ultralytics.engine.results import Masks

class ObjectDetector3D(Node):
    def __init__(self):
        super().__init__('object_detector_3d')
        self.declare_parameter('simulation_mode', True)

        self.model = YOLO('yolov8n-seg.pt')
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()

        self.target_class = 46  # 'banana' class index in YOLOv8

        # initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.target_pub = self.create_publisher(Point, 'target_position', 10)
        
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            1)
        
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/rgb')  # 使用简化的RGB话题
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/depth')
        
        # 设置同步器
        self.ts = message_filters.TimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.image_callback)
        
        self.get_logger().info('Object detector 3D node initialized')

        self.fx = 634.0862426757812
        self.fy = 634.0862426757812
        self.cx = 640.0
        self.cy = 360.0

    def camera_info_callback(self, msg):
        self.camera_model.fromCameraInfo(msg)

    def image_callback(self, rgb_msg, depth_msg):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            # Convert depth image according to its encoding
            if depth_msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
            elif depth_msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1').astype(np.float32)
                depth_image /= 1000.0  # Convert from mm to meters if needed
            else:
                self.get_logger().error(f'Unsupported depth encoding: {depth_msg.encoding}')
                return

            # Visualize depth image similar to RViz
            depth_vis = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)
            depth_vis = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = depth_vis.astype(np.uint8)
            cv2.imshow('Depth', depth_vis)
            cv2.waitKey(1)
            
            # Adjust image size for YOLOv8
            infer_shape = (736, 1280)
            results = self.model(rgb_image, imgsz=infer_shape)[0]
            depth_image = cv2.resize(depth_image, (infer_shape[1], infer_shape[0]), interpolation=cv2.INTER_NEAREST)
            rgb_image = cv2.resize(rgb_image, (infer_shape[1], infer_shape[0]), interpolation=cv2.INTER_LINEAR)
            print(f'Detected {len(results.boxes)} objects')

            for index, cls in enumerate(results[0].boxes.cls):
                class_index = int(cls.cpu().numpy())
                name = results[0].names[class_index]
                mask = results[0].masks.data.cpu().numpy()[index, :, :].astype(int)

                

                obj = depth_image[mask == 1]
                obj = obj[~np.isnan(obj)]
                avg_distance = np.mean(obj) if len(obj) else np.inf
                print(f'Class {name} has average distance {avg_distance:.2f} m')
                # avg_distance is deprecated...

                if int(cls) == self.target_class:
                    print("Shape of depth image:", depth_image.shape)
                    print("Shape of rgb image:", rgb_image.shape)

                    # Calculate the center of the mask and Get the depth value
                    xy_indices = results[0].masks.xy[0]
                    center_x, center_y = int(np.mean(xy_indices[:, 0])), int(np.mean(xy_indices[:, 1]))
                    print(f'Center of mask: ({center_x}, {center_y})')
                    depth = depth_image[center_y, center_x]

                    # Calculate 3D coordinates in camera frame
                    if not np.isnan(depth):
                        x = (center_x - self.cx) * depth / self.fx
                        y = (center_y - self.cy) * depth / self.fy
                        z = float(depth)
                        print(f'3D point in camera frame: ({x:.2f}, {y:.2f}, {z:.2f})') 
                        point_camera = tf2_geometry_msgs.PointStamped()
                        point_camera.header = rgb_msg.header
                        point_camera.point.x = x
                        point_camera.point.y = y
                        point_camera.point.z = z
                    
                    # Calculate 3D coordinates in base frame
                    try:
                        print(f"Find transform from {rgb_msg.header.frame_id} to base_link")
                        transform = self.tf_buffer.lookup_transform(
                            'base_link',
                            rgb_msg.header.frame_id,
                            rclpy.time.Time(), 
                            rclpy.duration.Duration(seconds=1.0)
                        )
                        point_base = tf2_geometry_msgs.do_transform_point(point_camera, transform)

                        target_point = Point()
                        target_point.x = point_base.point.x
                        target_point.y = point_base.point.y
                        target_point.z = point_base.point.z + 0.4 # Adjust Z position to be above the object 
                        
                        self.target_pub.publish(target_point)
                        self.get_logger().info(f'Published target position: ({target_point.x:.3f}, {target_point.y:.3f}, {target_point.z:.3f})')
                        
                        cv2.circle(rgb_image, (center_x, center_y), 5, (0, 255, 0), -1)
                        cv2.imshow('Detection', rgb_image)
                        cv2.waitKey(1)
                        
                    except Exception as e:
                        self.get_logger().error(f'TF transform failed: {str(e)}')
                    
                    break  # 只处理第一个检测到的目标
                    
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    detector = ObjectDetector3D()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()