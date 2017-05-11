import importlib

import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Empty

import intera_interface
from intera_interface import CHECK_VERSION

import redis, math, time

class TorqueDriver(object):
    def __init__(self, limb="right"):
        # control parameters
        self._rate = 2500.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # create our limb instance
        self._limb = intera_interface.Limb(limb)
        self._limb_names = ['right_j0', 'right_j1', 'right_j2', \
                            'right_j3', 'right_j4', 'right_j5', 'right_j6']

        # custom params
        self._springs = dict()
        self._damping = dict()
        self._start_angles = dict()

        # initialize redis instance
        self._redis = redis.StrictRedis(host='localhost', port=6379, db=0)

        # create cuff disable publisher ( Used to disable cuff button gravity comp mode )
        # cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        # self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

        # reset redis torque values so torque values aren't random
        self._redis.set('cs225a::robot::sawyer::actuators::fgc', '0 0 0 0 0 0 0')

        # get initial time
        self._start_time = rospy.Time.from_sec(time.time())

    def _update_forces(self):
        # Syncs between redis and ROS the joint torque and q, dq values

        # disable cuff interaction ( Used to disable cuff button gravity comp mode )
        # self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        for limb in self._limb_names:
            cmd[limb] = 0

        # record current angles/velocities
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()

        # Redis Keys
        # cs225a::robot::sawyer::actuators::fgc
        # cs225a::robot::sawyer::sensors::dq
        # cs225a::robot::sawyer::sensors::q

        # Reads and tokenizes to a list of torques (t0,t1,t2,t3,t4,t5,t6)
        redis_forces = self._redis.get('cs225a::robot::sawyer::actuators::fgc')
        if redis_forces is not None: # Check if read from redis correctly
            redis_forces = redis_forces.split(" ")
            if len(redis_forces) is 7: # Check if 7 forces read
                for i in range(0, 7):
                    force_float = float(redis_forces[i])
                    if not math.isnan(force_float): # Check if force is not NaN
                        cmd[self._limb_names[i]] = force_float # Set each force into cmd

        q_str_redis = ""
        dq_str_redis = ""
        # Set the q's and dq's to redis
        for joint in self._limb_names:
            q_str_redis += str(cur_pos[joint]) + " "
            dq_str_redis += str(cur_vel[joint]) + " "
        q_str_redis = q_str_redis[:-1] # Delete last character (space)
        dq_str_redis = dq_str_redis[:-1]

        self._redis.set('cs225a::robot::sawyer::sensors::q', q_str_redis)
        self._redis.set('cs225a::robot::sawyer::sensors::dq', dq_str_redis)

        # Set timestamp
        rt = rospy.Time.from_sec(time.time()) - self._start_time
        self._redis.set('cs225a::robot::sawyer::timestamp', str(rt.to_sec()))

        # command new joint torques
        self._limb.set_joint_torques(cmd)


    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral()

    def joint_torque_enable(self):
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and return to Position Control Mode
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break
            self._update_forces()
            control_rate.sleep()

    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()


def main():
    # Querying the parameter server to determine Robot model and limb name(s)
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
    robot_name = intera_interface.RobotParams().get_robot_name().lower().capitalize()

    # Starting node connection to ROS
    print("Initializing node... ")
    rospy.init_node("torque_driver_{0}".format(valid_limbs[0]))
    driver = TorqueDriver(limb=valid_limbs[0])
    # register shutdown callback
    rospy.on_shutdown(driver.clean_shutdown)
    driver.move_to_neutral()
    driver.joint_torque_enable()


if __name__ == "__main__":
    main()

