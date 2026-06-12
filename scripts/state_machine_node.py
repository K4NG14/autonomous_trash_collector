#!/usr/bin/env python3
import rospy
import actionlib
import math
import tf2_ros
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
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

        # 5. Recollida amb el braç: pausa la patrulla, mou el braç i la reprèn
        self.pickup_in_progress = False
        self.pickup_service = rospy.Service('~pickup_trash', Trigger, self.handle_pickup_trash)

        # 6. Entrega automàtica: quan YOLO detecta una escombraria, el robot
        #    desa la seva posició actual, va al contenidor corresponent,
        #    torna a la posició desada i reprèn la patrulla
        self.delivery_in_progress = False
        self.containers = rospy.get_param('~containers', {})
        if not self.containers:
            rospy.logwarn("⚠️ No hi ha cap contenidor configurat a ~containers "
                           "(config/containers.yaml). L'entrega automàtica "
                           "des de YOLO quedarà desactivada.")
        self.detection_sub = rospy.Subscriber('/detected_container', String,
                                               self.detection_callback, queue_size=1)

        # TF per saber on és el robot quan es detecta un objecte
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publica la posició on s'ha detectat l'objecte (per a RViz / logging)
        self.detection_pose_pub = rospy.Publisher('/detected_object_pose',
                                                    PoseStamped, queue_size=10)

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

                self._navigate_to(goal)

            if not rospy.is_shutdown():
                lista_waypoints.reverse()
                sentido_ida = not sentido_ida
                rospy.sleep(2.0)

    def _navigate_to(self, goal):
        """Envia un goal a move_base i espera. Si la navegació es preempta per
        una recollida amb el braç o una entrega a un contenidor, espera que
        acabi i reenvia el mateix goal."""
        while not rospy.is_shutdown():
            goal.target_pose.header.stamp = rospy.Time.now()
            self.client.send_goal(goal)
            self.client.wait_for_result()

            if self.pickup_in_progress or self.delivery_in_progress:
                rate = rospy.Rate(2)
                while (self.pickup_in_progress or self.delivery_in_progress) \
                        and not rospy.is_shutdown():
                    rate.sleep()
                continue

            return self.client.get_state()

    def handle_pickup_trash(self, req):
        rospy.loginfo("📦 Petició de recollida rebuda. Pausant patrulla...")
        self.pickup_in_progress = True
        try:
            self.client.cancel_goal()

            stop_msg = Twist()
            for _ in range(5):
                self.cmd_vel_pub.publish(stop_msg)
                rospy.sleep(0.05)

            rospy.wait_for_service('/arm_controller/pick', timeout=5.0)
            pick_srv = rospy.ServiceProxy('/arm_controller/pick', Trigger)
            result = pick_srv()

            rospy.loginfo(f"🦾 Recollida finalitzada: {result.message}")
            return TriggerResponse(success=result.success, message=result.message)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr(f"❌ Error movent el braç: {e}")
            return TriggerResponse(success=False, message=str(e))
        finally:
            rospy.loginfo("🔁 Reprenent patrulla...")
            self.pickup_in_progress = False

    def get_current_pose(self):
        """Retorna la pose actual del robot (PoseStamped en frame "map"),
        o None si la transformació encara no està disponible."""
        try:
            trans = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"❌ No s'ha pogut obtenir la pose del robot "
                          f"(map->base_footprint): {e}")
            return None

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = trans.transform.rotation
        return pose

    def _make_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        return goal

    def detection_callback(self, msg):
        container = msg.data

        if self.pickup_in_progress or self.delivery_in_progress:
            return

        if container not in self.containers:
            rospy.logwarn_throttle(
                10, f"⚠️ YOLO ha detectat '{container}' però no hi ha cap "
                "contenidor configurat amb aquest nom a containers.yaml.")
            return

        return_pose = self.get_current_pose()
        if return_pose is None:
            rospy.logwarn("⚠️ Ignorant la detecció: no es pot desar la "
                           "posició actual.")
            return

        self.detection_pose_pub.publish(return_pose)
        rospy.loginfo(
            f"🗑️ Detectat objecte per a '{container}'. Posició desada: "
            f"x={return_pose.pose.position.x:.2f}, "
            f"y={return_pose.pose.position.y:.2f}. "
            "Pausant patrulla i anant al contenidor...")

        self.delivery_in_progress = True
        try:
            self.client.cancel_goal()

            stop_msg = Twist()
            for _ in range(5):
                self.cmd_vel_pub.publish(stop_msg)
                rospy.sleep(0.05)

            # TODO: quan el braç estigui llest, recollir l'objecte aquí
            # (servei /arm_controller/pick) abans d'anar al contenidor.

            # 1. Anar al contenidor corresponent
            cont = self.containers[container]
            self.client.send_goal(
                self._make_goal(cont['x'], cont['y'], cont.get('yaw', 0.0)))
            self.client.wait_for_result()

            if self.client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo(f"✅ Arribat al contenidor '{container}'.")
                # TODO: quan el braç estigui llest, deixar l'objecte aquí
                # (servei /arm_controller/place).
            else:
                rospy.logerr(f"❌ No s'ha pogut arribar al contenidor "
                              f"'{container}'.")

            # 2. Tornar a la posició on s'ha detectat l'objecte
            rospy.loginfo("↩️ Tornant a la posició de detecció...")
            return_goal = MoveBaseGoal()
            return_goal.target_pose = return_pose
            self.client.send_goal(return_goal)
            self.client.wait_for_result()

            if self.client.get_state() != GoalStatus.SUCCEEDED:
                rospy.logwarn("⚠️ No s'ha pogut tornar exactament a la "
                               "posició de detecció. Reprenent patrulla "
                               "igualment.")
        finally:
            rospy.loginfo("🔁 Reprenent patrulla...")
            self.delivery_in_progress = False

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = StateMachineNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
