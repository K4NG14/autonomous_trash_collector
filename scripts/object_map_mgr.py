#!/usr/bin/env python3
import rospy

class ObjectMapManager:
    def __init__(self):
        rospy.init_node('object_map_mgr', anonymous=False)
        rospy.loginfo("Object Map Manager Node inicializado correctamente.")
        # Aquí guardaremos la base de datos en memoria de los objetos detectados

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObjectMapManager()
        node.spin()
    except rospy.ROSInterruptException:
        pass

