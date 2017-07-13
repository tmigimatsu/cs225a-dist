import redis
import math

import importlib
import rospy
import intera_interface
from intera_interface import CHECK_VERSION

class JointAngleDriver(object):
    def __init__(self, limb="right"):
        # control parameters
        self._rate = 2500.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        self._limb = intera_interface.Limb(limb)
        self._limb_names = ['right_j0', 'right_j1', 'right_j2', \
                            'right_j3', 'right_j4', 'right_j5', 'right_j6']
        # initialize redis instance
        self._redis = redis.StrictRedis(host='localhost', port=6379, db=0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

        # reset redis q values so they aren't random
        self._redis.set('cs225a::sawyer::sensors::q', '0 -0.7854 0 1.5708 0 -0.7854 0')

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral()

    def _update_angles(self):
        # Syncs between redis and ROS the joint angles

        # create our command dict
        cmd = dict()
        for limb in self._limb_names:
            cmd[limb] = 0

        # Redis Keys
        JOINT_KEY = "cs225a::sawyer::sensors::q"

        # Reads and tokenizes to a list of q angles (q0,q1,q2,q3,q4,q5,q6)
        redis_angles = self._redis.get(JOINT_KEY)
        if redis_angles is not None: # Check if read from redis correctly
            redis_angles = redis_angles.split(" ")
            if len(redis_angles) is 7: # Check if 7 angles read
                for i in xrange(7):
                    angle_float = float(redis_angles[i])
                    if not math.isnan(angle_float):
                        cmd[self._limb_names[i]] = angle_float

        # command new joint angles (WARNING: byspasses collision avoidance)
        self._limb.set_joint_positions(cmd)

    def joint_angle_enable(self):
        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and return to Position Control Mode
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        # loop at specified rate commanding new joint angles
        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("Joint angle controller failed to meet "
                             "specified control rate timeout.")
                break
            self._update_angles()
            control_rate.sleep()

    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting joint angle controller...")
        self._limb.exit_control_mode()


def main():
    # Querying the parameter server to determine Robot model and limb name(s)
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")

    # Starting node connection to ROS
    print("Initializing node... ")
    rospy.init_node("joint_angle_driver_{0}".format(valid_limbs[0]))
    driver = JointAngleDriver(limb=valid_limbs[0])
    # register shutdown callback
    rospy.on_shutdown(driver.clean_shutdown)
    driver.move_to_neutral()
    # start robot control loop
    driver.joint_angle_enable()


if __name__ == "__main__":
    main()
