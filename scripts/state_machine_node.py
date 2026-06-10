#!/usr/bin/env python3
import rospy
import actionlib
import math
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class StateMachineNode:
    def __init__(self):
        rospy.init_node('state_machine_node', anonymous=False)
        
        rospy.loginfo("Esperant connexió amb move_base...")
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Connectat a move_base! Llest per patrullar.")

        # Modifiquem els paràmetres de tolerància de ROS en calent
        rospy.set_param('/move_base/DWAPlannerROS/xy_goal_tolerance', 0.25) # 25 cm de marge
        rospy.set_param('/move_base/DWAPlannerROS/yaw_goal_tolerance', 3.14) # No importa la rotació final

        self.path_sub = rospy.Subscriber('/patrol/waypoints', Path, self.path_callback)
        self.is_patrolling = False

    def path_callback(self, msg):
        if self.is_patrolling:
            return 

        self.is_patrolling = True
        rospy.loginfo(f"Ruta rebuda! Conté {len(msg.poses)} waypoints.")
        self.execute_infinite_patrol(msg.poses)

    def execute_infinite_patrol(self, poses):
        forward_poses = poses
        backward_poses = poses[::-1]

        while not rospy.is_shutdown():
            rospy.loginfo("➡️ INICIANT PATRULLA D'ANADA...")
            self._navigate_waypoints(forward_poses)
            
            rospy.loginfo("⬅️ INICIANT PATRULLA DE TORNADA...")
            self._navigate_waypoints(backward_poses)

    def _navigate_waypoints(self, poses):
        for i in range(len(poses)):
            if rospy.is_shutdown():
                break
                
            rospy.loginfo(f"Navegant cap al waypoint {i+1}/{len(poses)}...")

            pose_stamped = poses[i]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            
            goal.target_pose.pose.position = pose_stamped.pose.position

            # Calcular cap a on mirar (Angle cap al següent waypoint)
            yaw = 0.0
            if i < len(poses) - 1:
                next_pose = poses[i+1]
                dx = next_pose.pose.position.x - pose_stamped.pose.position.x
                dy = next_pose.pose.position.y - pose_stamped.pose.position.y
                yaw = math.atan2(dy, dx)
            
            # Convertir Euler (yaw) a Quaternion per a ROS
            q = quaternion_from_euler(0, 0, yaw)
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]

            self.client.send_goal(goal)
            self.client.wait_for_result()
            
            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"✅ Waypoint {i+1} assolit.")
            else:
                rospy.logwarn(f"❌ Fallada al waypoint {i+1}. S'ha encallat. Saltant al següent...")

if __name__ == '__main__':
    try:
        node = StateMachineNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
