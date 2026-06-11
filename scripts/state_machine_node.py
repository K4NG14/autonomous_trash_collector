#!/usr/bin/env python3
import rospy
import actionlib
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class StateMachineNode:
    def __init__(self):
        rospy.init_node('state_machine_node')

        # 🔧 FIX: Crear el publicador de velocidad aquí para que la conexión Wi-Fi siempre esté "caliente"
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # REGISTRO DE EMERGENCIA
        rospy.on_shutdown(self.shutdown_hook)

        # 1. Configurar tolerancia
        rospy.set_param('/move_base/DWAPlannerROS/xy_goal_tolerance', 0.15)

        # 2. Cliente de Move Base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("⏳ Esperando al servidor move_base...")
        self.client.wait_for_server()
        rospy.loginfo("✅ Conectado a move_base.")

        # 3. Control de localización manual
        self.pose_received = False
        self.pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.loginfo("📌 SENSOR LISTO: Usa '2D Pose Estimate' en RViz para arrancar.")

        # 4. Suscriptor de ruta
        self.path_sub = rospy.Subscriber('/patrol/waypoints', Path, self.path_callback)
        self.path_received = False

    def pose_callback(self, msg):
        if not self.pose_received:
            self.pose_received = True
            rospy.loginfo("🎯 ¡Pose inicial detectada desde RViz!")

    def shutdown_hook(self):
        rospy.logwarn("🛑 [EMERGENCIA] Ctrl+C detectado. Frenando en seco...")
        self.client.cancel_all_goals()
        
        # 🔧 FIX: Usamos el publicador ya conectado para enviar el freno inmediato
        stop_msg = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_msg)
            rospy.sleep(0.05)
        rospy.loginfo("🤖 Robot detenido de forma segura.")

    def path_callback(self, msg):
        if self.path_received:
            return
        self.path_received = True

        rate = rospy.Rate(2)
        while not self.pose_received and not rospy.is_shutdown():
            rospy.logwarn_throttle(5, "⚠️ Robot congelado. Dale una pose en RViz...")
            rate.sleep()

        if rospy.is_shutdown():
            return

        rospy.loginfo("🔁 ¡Validación correcta! Iniciando patrulla...")
        lista_waypoints = list(msg.poses)
        lista_waypoints.reverse() 
        sentido_ida = True

        while not rospy.is_shutdown():
            for i, pose_stamped in enumerate(lista_waypoints):
                if rospy.is_shutdown():
                    break

                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = pose_stamped.pose.position.x
                goal.target_pose.pose.position.y = pose_stamped.pose.position.y
                goal.target_pose.pose.position.z = 0.0

                if i < len(lista_waypoints) - 1:
                    next_pose = lista_waypoints[i+1]
                    dx = next_pose.pose.position.x - pose_stamped.pose.position.x
                    dy = next_pose.pose.position.y - pose_stamped.pose.position.y
                    angle_yaw = math.atan2(dy, dx)
                    
                    q = quaternion_from_euler(0, 0, angle_yaw)
                    goal.target_pose.pose.orientation.x = q[0]
                    goal.target_pose.pose.orientation.y = q[1]
                    goal.target_pose.pose.orientation.z = q[2]
                    goal.target_pose.pose.orientation.w = q[3]
                else:
                    goal.target_pose.pose.orientation.w = 1.0

                self.client.send_goal(goal)
                self.client.wait_for_result()

            if not rospy.is_shutdown():
                lista_waypoints.reverse()
                sentido_ida = not sentido_ida
                rospy.sleep(2.0)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = StateMachineNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
