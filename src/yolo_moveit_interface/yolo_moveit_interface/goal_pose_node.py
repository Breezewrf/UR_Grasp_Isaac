#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
from rclpy.logging import get_logger
import copy
# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
import time

def plan_and_execute(
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
    ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
            logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])
            time.sleep(sleep_time)
            return True
        else:
            logger.error("Planning failed")
            time.sleep(sleep_time)
            return False
    

class PoseGoalNode(Node):
    def __init__(self, ur, ur_arm, gripper, logger):
        super().__init__('pose_goal_node')
        self.ur = ur
        self.ur_arm = ur_arm
        self.gripper = gripper
        self.logger = logger
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Point,
            'target_position',
            self.target_callback,
            10)
    
    def target_callback(self, msg):
            
        # 设置新的目标位置
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"  # 替换为实际的基座坐标系
        # 顶抓姿态：Z 轴朝下，无旋转
        pose_goal.pose.orientation.x = 0.0
        pose_goal.pose.orientation.y = 1.0
        pose_goal.pose.orientation.z = 0.0
        pose_goal.pose.orientation.w = 0.0
        pose_goal.pose.position.x = msg.x
        pose_goal.pose.position.y = msg.y
        pose_goal.pose.position.z = msg.z
        
       # Step 1: Move to approach pose
        self.ur_arm.set_start_state_to_current_state()
        self.ur_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool0")
        success = plan_and_execute(robot=self.ur, planning_component=self.ur_arm, logger=self.logger)
        if success:
            self.logger.info(f'Successfully moved to position: x={msg.x}, y={msg.y}, z={msg.z}')
        else:
            self.logger.error('Failed to reach target position, position might be unreachable')

        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name="open")
        gripper_status = plan_and_execute(robot=self.ur, planning_component=self.gripper, logger=self.logger, sleep_time=3.0)
        if gripper_status:
            self.logger.info('Gripper opened successfully')
        else:
            self.logger.error('Failed to open gripper, gripper might be unreachable or in an invalid state')

        # Step 2: Move straight down to grasp pose
        grasp_pose = copy.deepcopy(pose_goal)
        grasp_pose.pose.position.z -= 0.4
        grasp_pose.pose.position.z = max(grasp_pose.pose.position.z, 0.1)  # Ensure Z is not negative
        self.ur_arm.set_start_state_to_current_state()
        self.ur_arm.set_goal_state(pose_stamped_msg=grasp_pose, pose_link="tool0")
        success = plan_and_execute(robot=self.ur, planning_component=self.ur_arm, logger=self.logger)
        if not success:
            self.logger.error('Failed to reach grasp pose')
        else:
            self.logger.info(f'Successfully moved to grasp pose: x={grasp_pose.pose.position.x}, y={grasp_pose.pose.position.y}, z={grasp_pose.pose.position.z}')   
        
        # Step 3: Close gripper
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name="close")
        gripper_status = plan_and_execute(robot=self.ur, planning_component=self.gripper, logger=self.logger, sleep_time=3.0)
        if gripper_status:
            self.logger.info('Gripper closed successfully')
        else:
            self.logger.error('Failed to close gripper, gripper might be unreachable or in an invalid state')   

        
        

def main():
    rclpy.init()
    
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    ur = MoveItPy(
        node_name="vision_node",
    )
    
    ur_arm = ur.get_planning_component("arm")
    logger.info("MoveItPy instance created")

    gripper = ur.get_planning_component("gripper")
    logger.info("Gripper planning component initialized")
    
    mover = PoseGoalNode(ur, ur_arm, gripper, logger)
    rclpy.spin(mover)
    
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()