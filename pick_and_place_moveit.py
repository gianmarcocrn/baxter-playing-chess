#!/usr/bin/env python
import sys
import copy
import tf

from copy import deepcopy

import rospy
import rospkg

from std_msgs.msg import (
    Empty,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

import baxter_interface
import moveit_commander


class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        self._limb_name = limb  # string
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")

        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        # We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))

        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance

        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        # servo up from current pose
        self._group.set_pose_target(ik_pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def _servo_to_pose(self, pose):
        # servo down to release
        self._group.set_pose_target(pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        rospy.sleep(1.0)
        self.gripper_close()
        rospy.sleep(1.0)
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        rospy.sleep(1.0)
        self.gripper_open()
        rospy.sleep(1.0)
        # retract to clear object
        self._retract()



def add_pose_to_list(pose_list, coordinate, frame_of_reference):
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    pose_list.append(Pose(
        position=Point(x=frame_of_reference[coordinate][0], y=frame_of_reference[coordinate][1], z=frame_of_reference[coordinate][2]),
        orientation=overhead_orientation))


def perform_moves():
    pass

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ik_pick_and_place_moveit")

    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    # load_gazebo_models()
    # Remove models from the scene on shutdown
    # rospy.on_shutdown(delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15  # meters

    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    # NOTE: Gazebo and Rviz has different origins, even though they are connected. For this
    # we need to compensate for this offset which is 0.93 from the ground in gazebo to
    # the actual 0, 0, 0 in Rviz.

    spawn_pose = Pose(position=Point(x=0.68, y=0.68, z=0.7825))

    starting_pose = Pose(
        position=Point(x=0.7, y=0.135, z=0.35),
        orientation=overhead_orientation)

    pnp = PickAndPlaceMoveIt(limb, hover_distance)

    pnp.move_to_start(starting_pose)


    initial_position_map = rospy.get_param('piece_target_position_map')
    pieces_to_place = rospy.get_param('piece_names')
    print(type(pieces_to_place))
    pieces_xml = rospy.get_param('pieces_xml')
    print(type(pieces_xml))

    board_target_setup = ['', '', '**np****', '', '*******r', 'B****Q**', '', '']

    # getting pose for target positions
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    board_pose = Pose(Point(0.3,0.55,0.78), orient)
    frame_dist = 0.025
    origin_piece = 0.03125

    target_piece_position_map = dict()
    for row, each in enumerate(board_target_setup):
        for col, piece in enumerate(each):
            pose = deepcopy(board_pose)
            pose.position.x = board_pose.position.x + frame_dist + origin_piece + row * (2 * origin_piece)
            pose.position.y = board_pose.position.y - 0.55 + frame_dist + origin_piece + col * (2 * origin_piece)
            pose.position.z += 0.018
            target_piece_position_map[str(row)+str(col)] = [pose.position.x, pose.position.y, pose.position.z-0.93] #0.93 to compensate Gazebo RViz origin difference

    picking_poses = list()
    placing_poses = list()
    
    picking_coordinates = ['01', '07', '13', '62', '73']
    placing_coordinates = ['22', '47', '23', '52', '55']

    for coordinate in picking_coordinates:
        add_pose_to_list(picking_poses, coordinate, initial_position_map)
    
    for coordinate in placing_coordinates:
        add_pose_to_list(placing_poses, coordinate, target_piece_position_map)

    # srv_call = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    # for i in range(len(picking_poses)):
    #     print srv_call(pieces_to_place[i], pieces_xml[pieces_to_place[i][0]], "/", spawn_pose, "world")
    #     pick_pose = Pose(position=Point(x=spawn_pose.position.x, y=spawn_pose.position.y, z=-0.14), orientation=overhead_orientation)
    #     pnp.pick(pick_pose)
    #     print("Picking", pieces_to_place[i])
    #     place_pose = Pose(position=Point(x=picking_poses[i].position.x, y=picking_poses[i].position.y, z=-0.14), orientation=overhead_orientation)
    #     pnp.place(place_pose)
    #     print("Placing", pieces_to_place[i])

    
    for i in range(len(picking_coordinates)):
        pick_pose = Pose(position=Point(x=picking_poses[i].position.x, y=picking_poses[i].position.y, z=-0.14), orientation=overhead_orientation)
        pnp.pick(pick_pose)
        place_pose = Pose(position=Point(x=placing_poses[i].position.x, y=placing_poses[i].position.y, z=-0.14), orientation=overhead_orientation)
        pnp.place(place_pose)

    return 0


if __name__ == '__main__':
    sys.exit(main())
