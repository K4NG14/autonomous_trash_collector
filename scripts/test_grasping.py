#!/usr/bin/env python3
import sys
import rospy
import moveit_commander

def test_grasping():
    # Inicializar MoveIt y ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('aislado_grasping_test', anonymous=True)

    # Conectar con los grupos de articulaciones del OpenManipulator
    rospy.loginfo("Conectando con el brazo...")
    arm_group = moveit_commander.MoveGroupCommander("arm")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")

    # Configurar velocidad segura (0.1 = 10% de la velocidad máxima)
    arm_group.set_max_velocity_scaling_factor(0.3)
    
    try:
        # PASO 1: Abrir la pinza del todo (los valores son en metros)
        rospy.loginfo("1. Abriendo pinza...")
        gripper_group.set_joint_value_target([0.01, 0.01]) # Máxima apertura
        gripper_group.go(wait=True)
        rospy.sleep(1.0)

        # PASO 2: Bajar el brazo a la posición de recogida (Valores de las 4 articulaciones en radianes)
        rospy.loginfo("2. Bajando brazo a posición de captura...")
        # Valores aproximados para mirar hacia el suelo delante del robot
        posicion_abajo = [0.0, 0.5, 0.3, -0.7] 
        arm_group.set_joint_value_target(posicion_abajo)
        arm_group.go(wait=True)
        rospy.sleep(1.0)

        # PASO 3: Cerrar la pinza para atrapar la basura
        rospy.loginfo("3. Cerrando pinza...")
        # Un valor negativo o 0 fuerza el cierre hasta que detecta resistencia
        gripper_group.set_joint_value_target([-0.01, -0.01]) 
        gripper_group.go(wait=True)
        rospy.sleep(1.0)

        # PASO 4: Levantar el brazo a la posición de transporte ("Home")
        rospy.loginfo("4. Levantando el objeto (Posición Home)...")
        arm_group.set_named_target("home")
        arm_group.go(wait=True)
        rospy.sleep(1.0)

        rospy.loginfo("✅ ¡Secuencia de Grasping completada con éxito!")

    except Exception as e:
        rospy.logerr(f"Error durante el movimiento: {e}")

    # Apagar limpiamente
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    test_grasping()
