import redis
import math
import time

import importlib
import rospy
import intera_interface
from intera_interface import CHECK_VERSION


class RosRedisPositionDriver(object):
    def __init__(self, limb="right"):
        # Constants
        self._key_command_positions = "cs225a::sawyer::sensors::q"
        self._key_timestamp         = "cs225a::sawyer::timestamp"
        self._dof                   = 7
        self._control_rate          = 2500.0 # Hz
        self._timeout_missed_cmds   = 20.0 # Missed cycles before triggering timeout
        self._joint_names           = ["{0}_j{1}".format(limb, i) for i in range(self._dof)]

        # Initialize limb interface
        self._limb = intera_interface.Limb(limb)

        # Initialize redis instance
        self._redis = redis.StrictRedis(host="127.0.0.1", port=6379, db=0)

        # Reset redis q values so they aren't random
        self._redis.set(self._key_command_positions, "0 -0.7854 0 1.5708 0 -0.7854 0")

        # Verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

        # Get initial time
        self._start_time = rospy.Time.from_sec(time.time())

    def _update_command_positions(self):
        """
        Syncs between redis and ROS the joint angles
        """

        # Read the list of commanded joint angles (q0,q1,q2,q3,q4,q5,q6) from Redis
        str_command_positions = self._redis.get(self._key_command_positions)
        if str_command_positions is None:
            print("ERROR: Redis key " + self._key_command_positions + " not found.")
            return

        # Parse string returned by Redis
        try:
            command_positions = [float(q) for q in str_command_positions.split(" ")]
        except Exception as e:
            print("ERROR: Could not parse Redis key " self._key_command_positions)
            print(e)
            return

        # Error checking
        if len(command_positions) != self._dof:
            print("ERROR: Redis key " + self._key_command_positions + " must be of length " + self._dof + ".")
            return

        if any(math.isnan(q) for q in command_positions):
            print("ERROR: NaN joint angles: " + str_command_positions + ".")
            return

        # Set timestamp
        rt = rospy.Time.from_sec(time.time()) - self._start_time
        self._redis.set(self._key_timestamp, str(rt.to_sec()))

        # Command new joint angles (WARNING: byspasses collision avoidance)
        cmd = {link: q for (link, q) in zip(self._joint_names, command_positions)}
        self._limb.set_joint_positions(cmd)

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral()

    def run(self):
        # Set control rate
        control_rate = rospy.Rate(self._control_rate)

        # For safety purposes, set the control rate command timeout. If the
        # specified number of command cycles are missed, the robot will timeout
        # and return to Position Control Mode.
        self._limb.set_command_timeout((1.0 / self._control_rate) * self._num_missed_cmds)

        # Loop at specified rate commanding new joint angles
        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("Joint position controller failed to meet "
                             "specified control rate timeout.")
                break
            self._update_command_positions()
            control_rate.sleep()

    def shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting joint angle controller...")
        self._limb.exit_control_mode()


def main():
    # Query the parameter server to determine Robot model and limb name(s)
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. Exiting."), "ERROR")

    # Start node connection to ROS
    print("Initializing node... ")
    rospy.init_node("joint_angle_driver_{0}".format(valid_limbs[0]))

    # Initialize driver
    driver = RosRedisPositionDriver(limb=valid_limbs[0])

    # Register shutdown callback
    rospy.on_shutdown(driver.shutdown)

    # Start robot control loop
    driver.move_to_neutral()
    driver.run()


if __name__ == "__main__":
    main()
