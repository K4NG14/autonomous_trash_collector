#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from std_srvs.srv import Trigger, TriggerResponse

GRIPPER_OPEN = [0.01, 0.01]
GRIPPER_CLOSED = [-0.01, -0.01]

# Articulacions del braç (radians) per agafar un objecte de terra davant del robot
PICK_JOINTS = [0.0, 0.5, 0.3, -0.7]

# Articulacions del braç (radians) per deixar l'objecte al contenidor.
# Placeholder: cal calibrar-ho segons la posició real del contenidor respecte al robot.
PLACE_JOINTS = [1.57, 0.3, 0.2, -0.5]

ARM_VELOCITY_SCALING = 0.3


class ArmController:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('arm_controller', anonymous=False)

        rospy.loginfo("Connectant amb el braç i la pinça (MoveIt)...")
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.arm.set_max_velocity_scaling_factor(ARM_VELOCITY_SCALING)

        rospy.Service('~go_home', Trigger, self._wrap(self.go_home))
        rospy.Service('~open_gripper', Trigger, self._wrap(self.open_gripper))
        rospy.Service('~close_gripper', Trigger, self._wrap(self.close_gripper))
        rospy.Service('~pick', Trigger, self._wrap(self.pick_routine))
        rospy.Service('~place', Trigger, self._wrap(self.place_routine))

        rospy.loginfo("Arm Controller llest. Serveis: ~go_home, ~open_gripper, "
                       "~close_gripper, ~pick, ~place")

    def _wrap(self, fn):
        def handler(_req):
            try:
                fn()
                return TriggerResponse(success=True, message="ok")
            except Exception as e:
                rospy.logerr(f"Arm controller error: {e}")
                return TriggerResponse(success=False, message=str(e))
        return handler

    def open_gripper(self):
        self.gripper.set_joint_value_target(GRIPPER_OPEN)
        self.gripper.go(wait=True)

    def close_gripper(self):
        self.gripper.set_joint_value_target(GRIPPER_CLOSED)
        self.gripper.go(wait=True)

    def go_home(self):
        self.arm.set_named_target("home")
        self.arm.go(wait=True)

    def pick_routine(self):
        rospy.loginfo("🦾 Pick: obrint pinça...")
        self.open_gripper()

        rospy.loginfo("🦾 Pick: baixant a posició de recollida...")
        self.arm.set_joint_value_target(PICK_JOINTS)
        self.arm.go(wait=True)

        rospy.loginfo("🦾 Pick: tancant pinça...")
        self.close_gripper()

        rospy.loginfo("🦾 Pick: tornant a posició de transport (home)...")
        self.go_home()

    def place_routine(self):
        rospy.loginfo("🦾 Place: movent cap al contenidor...")
        self.arm.set_joint_value_target(PLACE_JOINTS)
        self.arm.go(wait=True)

        rospy.loginfo("🦾 Place: obrint pinça per deixar l'objecte...")
        self.open_gripper()

        rospy.loginfo("🦾 Place: tornant a posició de transport (home)...")
        self.go_home()

    def spin(self):
        rospy.spin()
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        ArmController().spin()
    except rospy.ROSInterruptException:
        pass
