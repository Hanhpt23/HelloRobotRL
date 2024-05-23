#!/usr/bin/env python3

# Import modules
import rospy
import time

# Import the FollowJointTrajectoryGoal from the control_msgs.msg package to
# control the Stretch robot
from control_msgs.msg import FollowJointTrajectoryGoal

# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories
from trajectory_msgs.msg import JointTrajectoryPoint

# Import hello_misc script for handling trajectory goals with an action client
import hello_helpers.hello_misc as hm

class MultiPointCommand(hm.HelloNode):
    """
    A class that sends multiple joint trajectory goals to the stretch robot.
    """
    def __init__(self):
        """
        Function that initializes the inherited hm.HelloNode class.
        :param self: The self reference.
        """
        hm.HelloNode.__init__(self)

    def issue_multipoint_command(self):
        """
        Function that makes an action call and sends multiple joint trajectory goals
        to the joint_lift, wrist_extension, joint_wrist_yaw, and joint_head_pan.
        :param self: The self reference.
        """
        # Create trajectory points with positions for all joints, including the fourth joint
        point0 = JointTrajectoryPoint()
        point0.positions = [0.2, 0.0, 3.4, 0.0]  # Fourth joint default to 0.0
        point0.velocities = [0.2, 0.2, 2.5, 0.0]  # Fourth joint default to 0.0
        point0.accelerations = [1.0, 1.0, 3.5, 0.0]  # Fourth joint default to 0.0

        point1 = JointTrajectoryPoint()
        point1.positions = [0.3, 0.1, 2.0, 0.0]

        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 0.2, -1.0, 0.0]

        point3 = JointTrajectoryPoint()
        point3.positions = [0.3, 0.3, 0.0, 0.0]

        point4 = JointTrajectoryPoint()
        point4.positions = [0.3, 0.2, 1.0, -0.2]

        point5 = JointTrajectoryPoint()
        point5.positions = [0.3, 0.2, 1.0, 0]  # Setting the fourth joint at the end

        # Set trajectory_goal as a FollowJointTrajectoryGoal and define
        # the joint names as a list
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 'joint_gripper_finger_left']

        # Then trajectory_goal.trajectory.points is defined by a list of the joint
        # trajectory points
        trajectory_goal.trajectory.points = [point0, point3, point4, point5]

        # Specify the coordinate frame that we want (base_link) and set the time to be now
        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        # Make the action call and send the goal. The last line of code waits
        # for the result before it exits the python script
        self.trajectory_client.send_goal(trajectory_goal)
        rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()

    def main(self):
        """
        Function that initiates the multipoint_command function.
        :param self: The self reference.
        """
        # The arguments of the main function of the hm.HelloNode class are the
        # node_name, node topic namespace, and boolean (default value is true)
        hm.HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)
        rospy.loginfo('issuing multipoint command...')
        self.issue_multipoint_command()
        time.sleep(2)

if __name__ == '__main__':
    try:
        # Instantiate a `MultiPointCommand()` object and execute the main() method
        node = MultiPointCommand()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
