#!/usr/bin/env python

# Import modules
import math
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

from std_msgs.msg import Int32
from std_msgs.msg import String

from sensor_msgs.msg import JointState

sample_move_object = 'biscuts'
excluded = \
        ['twin_table',
        'dropbox',
        'short_table']

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    cloud = outlier_filter_pcl(pcl_msg)
    # cloud = passthrough_filter(cloud)
    cloud = vox_filter(cloud)
    cluster_indices, white_cloud, extracted_inliers_objects = ransac_filter(cloud)
    detected_objects = classify_clusters(cluster_indices, white_cloud, extracted_inliers_objects)
    initialize_pr2_move_commands(detected_objects)


def outlier_filter_pcl(pcl_data):

    # Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_data)

    # Much like the previous filters, we start by creating a filter object:
    outlier_filter = cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(1.5)

    # Set threshold scale factor
    x = 1

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    return outlier_filter.filter()


def vox_filter(cloud_filtered):

    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud_filtered.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    LEAF_SIZE = .009

    # Set the voxel (or leaf) size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    return vox.filter()


def passthrough_filter(cloud_filtered):
    # PassThrough Filter - z axis
    # Create a PassThrough filter object.
    passthrough_z = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough_z.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough_z.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough_z.filter()

    # PassThrough Filter - y axis
    # Create a PassThrough filter object.
    passthrough_y = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'y'
    passthrough_y.set_filter_field_name(filter_axis)
    axis_min = -0.5
    axis_max = 0.5
    passthrough_y.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud.
    return passthrough_y.filter()


def ransac_filter(cloud_filtered):

    # RANSAC plane segmentation
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance
    # for segmenting the table
    max_distance = .01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract inliers

    # Extract Table
    extracted_inliers_table = cloud_filtered.extract(inliers, negative=False)

    # Extract Objects
    extracted_inliers_objects = cloud_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(extracted_inliers_objects)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(25000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(extracted_inliers_objects)
    # ros_cloud_table = pcl_to_ros(extracted_inliers_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_cluster_pub.publish(ros_cluster_cloud)

    return cluster_indices, white_cloud, extracted_inliers_objects


# Exercise-3 TODOs:
def classify_clusters(cluster_indices, white_cloud, extracted_inliers_objects):
    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = extracted_inliers_objects.extract(pts_list)

        # Compute the associated feature vector
        # convert the cluster from pcl to ROS using helper function
        ros_pcl_objects = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        chists = compute_color_histograms(ros_pcl_objects, using_hsv=True)
        normals = get_normals(ros_pcl_objects)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_pcl_objects
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    return detected_objects


def initialize_pr2_move_commands(detected_objects):

    try:
        publish_world_joint_controller()
    except rospy.ROSInterruptException:
        pass

    if len(detected_objects) > 0:
        print('no objects')
        # try:
        #     publish_collision_point_cloud(detected_objects, ros_cloud_table)
        # except rospy.ROSInterruptException:
        #     pass



        # excluded_move_object_pcl = {}
        # try:
        #     excluded_move_object_pcl = determine_move_params_yaml(detected_objects)
        # except rospy.ROSInterruptException:
        #     pass

        # try:
        #     move_pr2(excluded_move_object_pcl)
        # except rospy.ROSInterruptException:
        #     pass

    else:
        rospy.loginfo('There are no detected objects are not found')


# function to load parameters and request PickPlace service
def determine_move_params_yaml(detected_object_list):
    # get parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')

    test_scene_num = Int32
    test_scene_num.data = 4

    dict_list = []
    filename = 'output_4.yaml'

    move_dict = {}

    for i in range(0, len(object_list_param), 1):
        name = object_list_param[i]['name']
        group = object_list_param[i]['group']
        print('object_name:', name)

        for detected_object in detected_object_list:
            label = detected_object.label
            print('current label: ' + label)
            if name == label:
                points_arr = ros_to_pcl(detected_object.cloud).to_array()
                pick_pose = extract_pick_pose(np.mean(points_arr, axis=0)[:3])
                place_pose = extract_place_pose(group, dropbox_list_param)
                arm_name = extract_arm_name(group)
                obj_name = extract_object_name(name)
                yaml_dict = make_yaml_dict(test_scene_num, arm_name, obj_name, pick_pose, place_pose)
                dict_list.append(yaml_dict)
            if label != sample_move_object:
                # Exclude the object that we have specified to be moved and save off the dictionary to use when we
                # publish the collision point cloud
                move_dict = yaml_dict

    send_to_yaml(filename, dict_list)

    return move_dict


def move_pr2(move_dict):
    # Wait for 'pick_place_routine' service to come up
    rospy.wait_for_service('pick_place_routine')
    try:
        pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

        # Insert your message variables to be sent as a service request
        resp = pick_place_routine(move_dict["test_scene_num"], move_dict["object_name"],  move_dict["arm_name"], move_dict["pick_pose"], move_dict["place_pose"])

        print ("Response: ",resp.success)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    return


# helper function to create the pick pose object
def extract_pick_pose(centroid):
    pick_pose = Pose()
    pick_pose.position.x = np.asscalar(centroid[0])
    pick_pose.position.y = np.asscalar(centroid[1])
    pick_pose.position.z = np.asscalar(centroid[2])
    return pick_pose


# helper function to create the place pose object which includes finding the matching group items from the
# droplist params
def extract_place_pose(group_name, params):
    place_pose = Pose()
    for i in range(0, len(params), 1):
        param_group = params[i]['group']
        print('param position: ', params[i]['position'])
        if group_name == param_group:
            place_pose.position.x = params[i]['position'][0]
            print('type position x:', type(place_pose.position.x))
            place_pose.position.y = params[i]['position'][1]
            print('type position y:', type(place_pose.position.y))
            place_pose.position.z = params[i]['position'][2]
            print('type position z:', type(place_pose.position.z))
    return place_pose


# helper function to create a properly structured object name object
def extract_object_name(name):
    object_name = String()
    object_name.data = name
    return object_name


# helper function to determine the correct arm name based upon the group string
def extract_arm_name(group):
    arm_name = String()
    right_arm = 'green'
    left_arm = 'red'
    if group == right_arm :
        arm_name.data = 'right'
    elif group == left_arm :
        arm_name.data = 'left'
    return arm_name


# challenge - publish pcl collision list
def publish_collision_point_cloud(detected_objects, table):
    detected_object_collision_list = []

    for detected_object in detected_objects:
        if sample_move_object != detected_object.label:
            ros_pcl_object = detected_object.cloud
            detected_object_collision_list.append(ros_pcl_object)

    ros_pcl_table = pcl_to_ros(table)
    detected_object_collision_list.append(ros_pcl_table)
    collision_pcl_pub.publish(detected_object_collision_list)
    return


# challenge - publish the joint controller angle to rotate the robot
def publish_world_joint_controller():
    pr2_world_joint_controller_pub = rospy.Publisher('/pr2/world_joint_controller/command', Float64, queue_size=10)

    joint_state = rospy.wait_for_message('/pr2/joint_states', JointState)

    # print('joint_state', joint_state)

    joint_angle = radians_to_degrees(joint_state.position[19])

    print('joint_angle in degrees:', joint_angle)

    while True:
        if joint_angle < 358 and joint_angle > 2:
            print('rotation angle between 360 and 0')
            break
        elif joint_angle <= 1:
            print('zero or less rotation angle')
            pr2_world_joint_controller_pub.publish(degrees_to_radians(360))
        elif joint_angle >= 359:
            print('360 or greater rotation angle')
            pr2_world_joint_controller_pub.publish(degrees_to_radians(0))
        break

def at_goal(curr_angle, goal):
    tolerance = .05
    result = abs(pos_j1 - goal_j1) <= abs(tolerance)
    result = result and abs(pos_j2 - goal_j2) <= abs(tolerance)
    return result


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"] = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict


# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


def radians_to_degrees(angle):
    return angle * 180/np.pi


def degrees_to_radians(degrees):
    return degrees * np.pi / 180


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    collision_pcl_pub = rospy.Publisher('/pr2/3D_map/points', PointCloud2, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
