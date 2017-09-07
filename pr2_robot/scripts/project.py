#!/usr/bin/env python

# Import modules
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


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    print "sending to yaml", dict_list
    for d in dict_list:
        print "test_scene_num", type(d["test_scene_num"]), "arm_name", type(d["arm_name"]), "object_name", type(d["object_name"]), "pick_pose", type(d["pick_pose"]), "place_pose", type(d["place_pose"])

    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


def statistical_outlier_removal(cloud):
    # Much like the previous filters, we start by creating a filter object: 
    outlier_filter = cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(50)

    # Set threshold scale factor
    x = 0.9

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()
    return cloud_filtered


def voxel_downsample(cloud):
    """ Voxel Grid filter

        Args:
            cloud (PointCloud_PointXYZRGB): A point cloud

        Returns: 
            PointCloud_PointXYZRGB: A downsampled point cloud
    """
    # Create a VoxelGrid filter object for our input point cloud
    vox = cloud.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    LEAF_SIZE = 0.005

    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()
    return cloud_filtered


def apply_passthrough_filter(cloud, axis, axis_min, axis_max):
    """ Apply a passthrough filter to a cloud

        Args:
            cloud (PointCloud_PointXYZRGB): A point cloud

        Returns:
            PointCloud_PointXYZRGB: A filtered point cloud
    """
    # Create a PassThrough filter object.
    passthrough = cloud.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = axis
    passthrough.set_filter_field_name(filter_axis)
    #axis_min = 0.6
    #axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough.filter()
    return cloud_filtered


def ransac(cloud, sacmodel):
    """ Segments a cloud using a sac model

        Args:
            cloud (PointCloud_PointXYZRGB): A point cloud
            sacmodel (pcl.SACMODEL): A model points will be fit to

        Returns:
            A set of inliers and coefficients
    """
    # Create the segmentation object
    seg = cloud.make_segmenter()

    # Set the model you wish to fit 
    seg.set_model_type(sacmodel)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()
    return inliers, coefficients


def euclidean_clustering(cloud):
    white_cloud = XYZRGB_to_XYZ(cloud)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()

    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(25000)

    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)

    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    return cluster_indices, white_cloud


def color_clusters(cluster_indices, white_cloud):
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    return cluster_cloud


# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)
    #filename = 'pcl_data.pcd'
    #pcl.save(pcl_data, filename)

    # Statistical Outlier Filtering
    cloud_filtered = statistical_outlier_removal(pcl_data)
    #filename = 'statistical_outlier_removal.pcd'
    #pcl.save(cloud_filtered, filename)

    # Voxel Grid Downsampling
    cloud_filtered = voxel_downsample(cloud_filtered)
    #filename = 'voxel_downsampled.pcd'
    #pcl.save(cloud_filtered, filename)

    # PassThrough Filter along z
    axis_min = 0.6
    axis_max = 1.1
    cloud_filtered = apply_passthrough_filter(cloud_filtered, 'z', axis_min, axis_max)
    #filename = 'pass_through_filtered.pcd'
    #pcl.save(cloud_filtered, filename)

    # PassThrough Filter along y
    axis_min = -0.5
    axis_max = 0.5
    cloud_filtered = apply_passthrough_filter(cloud_filtered, 'y', axis_min, axis_max)
    filename = 'pass_through_filtered_y.pcd'
    pcl.save(cloud_filtered, filename)

    # RANSAC Plane Segmentation
    inliers, coefficients = ransac(cloud_filtered, pcl.SACMODEL_PLANE)

    # Extract inliers and outliers
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    #filename = 'extracted_inliers.pcd'
    #pcl.save(extracted_inliers, filename)

    extracted_outliers = cloud_filtered.extract(inliers, negative=True)
    #filename = 'extracted_outliers.pcd'
    #pcl.save(extracted_outliers, filename)


    cloud_table = extracted_inliers
    cloud_objects = extracted_outliers

    # Euclidean Clustering
    cluster_indices, white_cloud = euclidean_clustering(cloud_objects)

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_cloud = color_clusters(cluster_indices, white_cloud)
    filename = 'colored_cluster_cloud.pcd'
    pcl.save(cluster_cloud, filename)

    # Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        # Compute the associated feature vector
        ros_cluster = pcl_to_ros(pcl_cluster)
        sample_cloud = ros_cluster
        chists = compute_color_histograms(sample_cloud, using_hsv=True)
        normals = get_normals(sample_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    detected_objects_list = detected_objects
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    dict_list = []

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables
    test_scene_num = Int32()
    test_scene_num.data = 3

    """ labels = []
    centroids = [] # to be list of tuples (x, y, z)
    for object in object_list:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])
    """


    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for object in object_list:

        # TODO: Get the PointCloud for a given object and obtain it's centroid
        points_arr = ros_to_pcl(object.cloud).to_array()
        c = np.mean(points_arr, axis=0)[:3]
        centroid = map(np.asscalar, c)
        print "type(centroid)", type(centroid), "[0]", type(centroid[0]), "[1]", type(centroid[1]), "[2]", type(centroid[2])
        #centroids.append(np.mean(points_arr, axis=0)[:3])

        object_name = String()
        object_name.data = str(object.label)

        # TODO: Create 'place_pose' for the object
        group = None
        for o in object_list_param:
            if object.label == o['name']:
                group = o['group']
                print "for ", o['name'], "group found",group

        object_arm_name = String()
        place_pose = Pose()
        for box in dropbox_list_param:
            if group == box['group']:
                # Assign the arm to be used for pick_place
                object_arm_name.data = box['name']

                place_pose.position.x = box['position'][0]
                place_pose.position.y = box['position'][1]
                place_pose.position.z = box['position'][2]
                print "type(place_pose.x,y,z): (", type(place_pose.position.x), ",", type(place_pose.position.y), ",", type(place_pose.position.z), ")"

        pick_pose = Pose()
        pick_pose.position.x = centroid[0]
        pick_pose.position.y = centroid[1]
        pick_pose.position.z = centroid[2]

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, object_arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, object_arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    yaml_filename = "output_" + str(test_scene_num.data) + ".yaml"
    send_to_yaml(yaml_filename, dict_list)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    #pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)
    pcl_cam_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

