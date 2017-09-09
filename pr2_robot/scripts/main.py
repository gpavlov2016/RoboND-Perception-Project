#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from visualization_msgs.msg import Marker
from pr2_robot.msg import DetectedObjectsArray
from pr2_robot.msg import DetectedObject

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
from marker_tools import*
import pcl


from features import compute_color_histograms
from features import compute_normal_histograms


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
  ec.set_MinClusterSize(100)
  ec.set_MaxClusterSize(2000)
  # Search the k-d tree for clusters
  ec.set_SearchMethod(tree)
  # Extract indices for each of the discovered clusters
  cluster_indices = ec.Extract()

  #for cluster in cluster_indices:
  #  rospy.logerr("len(cluster_indices[i]) = %d", len(cluster))
  
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

def extract_features(sample_cloud):
  # Extract histogram features
  chists = compute_color_histograms(sample_cloud, using_hsv=True)
  normals = get_normals(sample_cloud)
  nhists = compute_normal_histograms(normals)
  features = np.concatenate((chists, nhists))
  
  return features


def object_recognition(cluster_indices, cloud_objects, white_cloud):
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # convert the cluster from pcl to ROS using helper function
        ros_cloud = pcl_to_ros(pcl_cluster)
        
        # Extract histogram features
        # complete this step just as is covered in capture_features.py
        features = extract_features(ros_cloud)
    
        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(features.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        #detected_objects_list.append(label)
        
        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))
    
        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cloud
        detected_objects.append(do)
    
        #rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects), detected_objects))
              
    return detected_objects

#objects is of type DetectedObject
def calc_centroids(objects):
  labels = []
  centroids = [] # to be list of tuples (x, y, z)
  for object in objects:
      labels.append(object.label)
      points_arr = ros_to_pcl(object.cloud).to_array()
      centroids.append(np.mean(points_arr, axis=0)[:3])
  return labels, centroids
  
  
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Pose

def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict
  

def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)  
  
# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
    pcl_cloud = ros_to_pcl(pcl_msg)

    # Voxel Grid Downsampling
    downsampled = downsample(pcl_cloud)

    # TODO: Statistical Outlier Filtering
    
    # PassThrough Filter
    table_cloud = passthrough(downsampled, 0.6, 1.1, 'z')

    # RANSAC Plane Segmentation
    table, objects = ransac(table_cloud)
        
    # Euclidean Clustering
    cluster_indices, white_cloud = segmentation(objects)     

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    segmented = segments_visuatlization(cluster_indices, white_cloud)
    
    # Convert PCL data to ROS messages and publish 
    pcl_objects_pub.publish(pcl_to_ros(objects))
    pcl_segmented_pub.publish(pcl_to_ros(segmented))
    pcl_table_pub.publish(pcl_to_ros(table))

    # Classify the clusters
    detected_objects = object_recognition(cluster_indices, objects, white_cloud)
    
    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    labels, centroids = calc_centroids(detected_objects)

    # get parameters
    object_list_param = rospy.get_param('/object_list')

    # get parameters
    dropbox_params = rospy.get_param('/dropbox')
    
    box_pos_dict = {}
    for dropbox_param in dropbox_params:
      name = dropbox_param['name']
      group = dropbox_param['group']
      position = dropbox_param['position']
      box_pos_dict[group] = position
    

    arm_dict = {'green':'right', 'red':'left'}
    dict_list = []
    for obj_param in object_list_param:
      name = obj_param['name']
      group = obj_param['group']
      for i in range(len(labels)):
        label = labels[i]
        centroid = centroids[i]
        if label == name:
          test_scene_num = Int32()
          test_scene_num.data = 1.0
          object_name = String()
          object_name.data = str(label)
          arm_name = String()
          arm_name.data = arm_dict[group]
          pick_pose = Pose()
          pick_pose.position.x = np.asscalar(centroid[0]) 
          pick_pose.position.y = np.asscalar(centroid[1]) 
          pick_pose.position.z = np.asscalar(centroid[2])
          place_pose = Pose()
          box_pos = box_pos_dict[group]
          place_pose.position.x = box_pos[0] 
          place_pose.position.y = box_pos[1] 
          place_pose.position.z = box_pos[2] 

          yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
          dict_list.append(yaml_dict)
          break
    
    send_to_yaml('output_1.yaml', dict_list)
    #send_to_yaml('output_2.yaml', dict_list)
    #send_to_yaml('output_3.yaml', dict_list)

    try:
        pr2_mover(detected_objects)
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
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    #pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)
    
    #Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", pc2.PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", pc2.PointCloud2, queue_size=1)
    pcl_segmented_pub = rospy.Publisher("/pcl_segmented", pc2.PointCloud2, queue_size=1)

    # create two publishers
    # Call them object_markers_pub and detected_objects_pub
    # Have them publish to "/object_markers" and "/detected_objects" with 
    # Message Types "Marker" and "DetectedObjectsArray" , respectively
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

    #Spin while node is not shutdown
    while not rospy.is_shutdown():
      rospy.spin()