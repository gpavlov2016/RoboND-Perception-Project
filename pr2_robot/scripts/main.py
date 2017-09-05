#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
#from sensor_stick.srv import GetNormals
#from sensor_stick.features import compute_color_histograms
#from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
#from sensor_stick.marker_tools import *
#from sensor_stick.msg import DetectedObjectsArray
#from sensor_stick.msg import DetectedObject
#pcfrom sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml
import sensor_msgs.point_cloud2 as pc2
from pcl_helper import *
import pcl


def downsample(cloud):
  # Create a VoxelGrid filter object for our input point cloud
  vox = cloud.make_voxel_grid_filter()
  
  # Choose a voxel (also known as leaf) size
  # Note: this (1) is a poor choice of leaf size   
  # Experiment and find the appropriate size!
  LEAF_SIZE = 0.01   
  
  # Set the voxel (or leaf) size  
  vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
  
  # Call the filter function to obtain the resultant downsampled point cloud
  cloud_filtered = vox.filter()
  return cloud_filtered
  
  #filename = 'voxel_downsampled.pcd'
  #pcl.save(cloud_filtered, filename)


def passthrough(cloud, min=0.6, max=1.1, axis='z'):
  # PassThrough filter
  # Create a PassThrough filter object.
  passthrough = cloud.make_passthrough_filter()
  
  # Assign axis and range to the passthrough filter object.
  passthrough.set_filter_field_name(axis)
  passthrough.set_filter_limits(min, max)
  
  # Finally use the filter function to obtain the resultant point cloud. 
  cloud_filtered = passthrough.filter()
  
  return cloud_filtered
  
  #filename = 'pass_through_filtered.pcd'
  #pcl.save(cloud_filtered, filename)


def ransac(cloud):
  # Create the segmentation object
  seg = cloud.make_segmenter()
  
  # Set the model you wish to fit 
  seg.set_model_type(pcl.SACMODEL_PLANE)
  seg.set_method_type(pcl.SAC_RANSAC)
  
  # Max distance for a point to be considered fitting the model
  # Experiment with different values for max_distance 
  # for segmenting the table
  max_distance = 0.01
  seg.set_distance_threshold(max_distance)
  
  # Call the segment function to obtain set of inlier indices and model coefficients
  inliers, coefficients = seg.segment()

  # Extract inliers
  table   = cloud.extract(inliers, negative=False)
  objects = cloud.extract(inliers, negative=True)
  
  return table, objects

  #filename = 'extracted_inliers.pcd'
  #pcl.save(extracted_inliers, filename)


def outlier_filter(cloud):
  # Much like the previous filters, we start by creating a filter object: 
  filter = cloud.make_statistical_outlier_filter()
  
  # Set the number of neighboring points to analyze for any given point
  filter.set_mean_k(50)
  
  # Set threshold scale factor
  x = 1.0
  
  # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
  filter.set_std_dev_mul_thresh(x)
  
  # Finally call the filter function for magic
  cloud_filtered = filter.filter()
  
  return cloud_filtered
  

from sklearn.cluster import DBSCAN  
def segmentation(cloud): 

  white_cloud = XYZRGB_to_XYZ(cloud)
  tree = white_cloud.make_kdtree()
  
  # Create a cluster extraction object
  ec = white_cloud.make_EuclideanClusterExtraction()
  # Set tolerances for distance threshold 
  # as well as minimum and maximum cluster size (in points)
  ec.set_ClusterTolerance(0.02)
  ec.set_MinClusterSize(150)
  ec.set_MaxClusterSize(2000)
  # Search the k-d tree for clusters
  ec.set_SearchMethod(tree)
  # Extract indices for each of the discovered clusters
  cluster_indices = ec.Extract()

  for cluster in cluster_indices:
    rospy.logerr("len(cluster_indices[i]) = %d", len(cluster))
  
  return cluster_indices, white_cloud
  
  
def segments_visuatlization(cluster_indices, white_cloud):
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
  
  return  cluster_cloud
   


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
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    pcl_cloud = ros_to_pcl(pcl_msg)
    downsampled = downsample(pcl_cloud)
    
    table_cloud = passthrough(downsampled, 0.6, 1.1, 'z')
    table, objects = ransac(table_cloud)
        
    cluster_indices, white_cloud = segmentation(objects)     

    segmented = segments_visuatlization(cluster_indices, white_cloud)
    
    # TODO: Statistical Outlier Filtering

    # TODO: Voxel Grid Downsampling

    # TODO: PassThrough Filter

    # TODO: RANSAC Plane Segmentation

    # TODO: Extract inliers and outliers

    # TODO: Euclidean Clustering

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    # TODO: Convert PCL data to ROS messages

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(pcl_to_ros(segmented))
    pcl_table_pub.publish(pcl_to_ros(table))

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)

        # Grab the points for the cluster

        # Compute the associated feature vector

        # Make the prediction

        # Publish a label into RViz

        # Add the detected object to the list of detected objects.

    # Publish the list of detected objects

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables

    # TODO: Get/Read parameters

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list

        # TODO: Get the PointCloud for a given object and obtain it's centroid

        # TODO: Create 'place_pose' for the object

        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file



if __name__ == '__main__':

    #ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    
    #Create Subscribers
    #pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)
    
    #Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", pc2.PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", pc2.PointCloud2, queue_size=1)

    # TODO: Load Model From disk

    # Initialize color_list
    get_color_list.color_list = []

    #Spin while node is not shutdown
    while not rospy.is_shutdown():
      rospy.spin()