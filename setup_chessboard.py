#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy

from pick_and_place_moveit import PickAndPlaceMoveIt

def add_pose_to_list(pose_list, coordinate, frame_of_reference):
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    pose_list.append(Pose(
        position=Point(x=frame_of_reference[coordinate][0], y=frame_of_reference[coordinate][1], z=frame_of_reference[coordinate][2]),
        orientation=overhead_orientation))

def main():
    rospy.init_node("setup_chessboard")
    limb = 'left'
    hover_distance = 0.15  # meters
    pnp = PickAndPlaceMoveIt(limb, hover_distance)

    spawn_pose = Pose(position=Point(x=0.68, y=0.68, z=0.7825))

    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    starting_pose = Pose(
        position=Point(x=0.7, y=0.135, z=0.35),
        orientation=overhead_orientation)

    pnp = PickAndPlaceMoveIt(limb, hover_distance)

    pnp.move_to_start(starting_pose)

    initial_position_map = rospy.get_param('piece_target_position_map')
    pieces_to_place = rospy.get_param('piece_names')
    pieces_xml = rospy.get_param('pieces_xml')

    picking_poses = list()
    picking_coordinates = ['01', '07', '13', '62', '73']

    for coordinate in picking_coordinates:
        add_pose_to_list(picking_poses, coordinate, initial_position_map)

    srv_call = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    for i in range(len(picking_poses)):
        print srv_call(pieces_to_place[i], pieces_xml[pieces_to_place[i][0]], "/", spawn_pose, "world")
        pick_pose = Pose(position=Point(x=spawn_pose.position.x, y=spawn_pose.position.y, z=-0.14), orientation=overhead_orientation)
        pnp.pick(pick_pose)
        print("Picking", pieces_to_place[i])
        place_pose = Pose(position=Point(x=picking_poses[i].position.x, y=picking_poses[i].position.y, z=-0.14), orientation=overhead_orientation)
        pnp.place(place_pose)
        print("Placing", pieces_to_place[i])
    
if __name__ == '__main__':
    sys.exit(main())