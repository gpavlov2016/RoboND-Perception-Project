[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

# Project Setup
For this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly
If you do not have an active ROS workspace, you can create one by:

```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the src directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Perception-Project.git
```
### Note: If you have the Kinematics Pick and Place project in the same ROS Workspace as this project, please remove the 'gazebo_grasp_plugin' directory from the `RoboND-Perception-Project/` directory otherwise ignore this note. 

Now install missing dependencies using rosdep install:
```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```
Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/models:$GAZEBO_MODEL_PATH
```

If you haven’t already, following line can be added to your .bashrc to auto-source all new terminals
```
source ~/catkin_ws/devel/setup.bash
```

To run the demo:
```sh
$ cd ~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/scripts
$ chmod u+x pr2_safe_spawner.sh
$ ./pr2_safe_spawner.sh
```
![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)



Once Gazebo is up and running, make sure you see following in the gazebo world:
- Robot

- Table arrangement

- Three target objects on the table

- Dropboxes on either sides of the robot


If any of these items are missing, please report as an issue on [the waffle board](https://waffle.io/udacity/robotics-nanodegree-issues).

In your RViz window, you should see the robot and a partial collision map displayed:

![demo-2](https://user-images.githubusercontent.com/20687560/2The8748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Proceed through the demo by pressing the ‘Next’ button on the RViz window when a prompt appears in your active terminal

The demo ends when the robot has successfully picked and placed all objects into respective dropboxes (though sometimes the robot gets excited and throws objects across the room!)

Close all active terminal windows using **ctrl+c** before restarting the demo.

You can launch the project scenario like this:
```sh
$ roslaunch pr2_robot pick_place_project.launch
```

# Vision Pipeline
Object recognition in this project is based on a point cloud output of a RGBD camera. The script `main.py` in `pr2_robot/scripts/` contains most of the code implementing the pipeline. The RGBD cloud point data is received by subscribing to `/pr2/world/points` topic. Following sections elaborate on each step in the pipeline.

## Filtering and Isolation
The scene contains a table and objects, therefore it is necessary to separate the table from the objects. This is done in a function `ransac()` using RANSAC plane fitting and filtering out the inliers (that belong to the table that we are not interested in).

## Euclidian Clustering
This step identifies groups of points which belong to the same object. In this case a spatial distance between points was used as a metric for segmentation. The step is performed in `segmentation()` function.

## Object Recognition
Object recognition is done using Support Vector Machine classifier (SVM) that was prior trained on each one of the object models by sampling cloud points in different poses. Following are more details about training and classification.

### Features 
There are two types of features that are extracted from the cloud point of an object:
* histograms of colors - each one of the HSV channels produces a 32-bin histogram of channel values ranging 0-256 
* histogram of normals - for each one of the HSV channels normals to the object's surface are calculated and produce a 32-bin histogram of channel values ranging 0-256

In both cases the histograms are normalized to sum to 1 and concatenated to create a feature vector. The code responsible for this extraction is in `main.py` under the function `extract_features`. This function uses helper function in the file `features.py`.

### Model Training
To train the model 100 samples of each class were collected as point cloud data from RGBD camera, after spawning a model of the object in random position in Gazebo. The classifier is SVM (Support Vector Machine) with linear kernel and it was trained on a training set comprising on a random 80% of the data. The rest is test data on which the accuracy tests were performed. The accuracy on the test set was above 90%. The code responsible for training the classifier is located in `train_svm.py`.
![confusion_normalized](confusion_normalized.png)
![confusion_matrix](confusion_matrix.png)

