#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates

# This is hard-coded to block for this exercise, yet you can make the script general by adding cmd line arguments
input_linknames = rospy.get_param("piece_names")

# Global variable where the object's pose is stored
poses = None


def get_links_gazebo(link_states_msg):
    # Call back to retrieve the object you are interested in
    global input_linknames
    global poses
    poses = {'world': link_states_msg.pose[0]} # get world link
    for (link_idx, link_name) in enumerate(link_states_msg.name):
        modelname = link_name.split('::')[0]
        if modelname in input_linknames:
            poses[modelname] = link_states_msg.pose[link_idx]


def main():
    rospy.init_node('gazebo2tfframe')

    # Create TF broadcaster -- this will publish a frame give a pose
    tfBroadcaster = tf.TransformBroadcaster()
    # SUbscribe to Gazebo's topic where all links and objects poses within the simulation are published
    rospy.Subscriber('gazebo/link_states', LinkStates, get_links_gazebo)

    rospy.loginfo('Spinning')
    global poses
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if poses is not None:
            for piece_name in input_linknames:
                pose = poses[piece_name]
                pos = pose.position
                ori = pose.orientation
                rospy.loginfo(pos)
                # Publish transformation given in pose
                tfBroadcaster.sendTransform((pos.x, pos.y, pos.z - 0.93), (ori.x, ori.y, ori.z, ori.w), rospy.Time.now(), piece_name, 'world')
                rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()