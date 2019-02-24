## Project: Perception Pick & Place
### Project 2 submission by Lewis Siempelkamp for Udacity's Robotics NanoDegree Term 1 
### February 2019

---
[//]: # (Image References)

[image1]: ./images/default_gzclient_camera(1)-2019-02-24T00_02_38.452216.jpg
[image2]: ./images/rviz_screenshot_2019_02_23-23_57_53.png
[image3]: ./images/rviz_screenshot_2019_02_23-23_58_34.png
[image4]: ./images/rviz_screenshot_2019_02_24-00_00_22.png
[image5]: ./images/rviz_screenshot_2019_02_24-00_00_44.png
[image6]: ./images/rviz_screenshot_2019_02_24-00_01_24.png
[image7]: ./images/rviz_screenshot_2019_02_24-00_03_37.png
[image8]: ./images/figure_2-2.png
[image9]: ./images/rviz_screenshot_2019_02_24-01_01_57.png

# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
  1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented
  2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  
  3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
  
  Each of the three exercises were completed to process the incoming RGB-D data stream from the virtual camera in the gazebo simulator.
  The pipeline was fully implemented in the project submission python file: [project_template.py](/pr2_robot/scripts/project_template.py)
  The following images demonstrate the process of the data processing pipeline necessary for performing object recognition from a raw, noisy data stream:
  
  The environment:
  
  ![the environment][image2]
  
  The point cloud from the raw, noisy RGB-D data stream of the environment:
  
  ![raw RGB-D][image3]
  
  The point cloud after filtering out the noise using make_statistical_outlier_filter(), Voxel Grid Downsampling (resolution reduction), Passthrough Filtering (cropping in Z and Y axes), and RANSAC plane segmentation (removing those points that together resemble a plane):
  
  ![filtered cloud][image4]
  
  The point cloud made into indexable clusters using make_EuclidianClusterExtraction():
  
  ![clustered cloud][image5]
  
  Object recognition performed on the final filtered/clustered RGB-D point cloud data using Support Vector Classification (SVC):
  
  ![objects recognized][image6]
  
  Outside of this pipeline a training set of RGB-D data was generated for a catalog of objects and used to train a SVC model that could be used in the pipeline for the object recognition step. The training set was generatedin part by the modified [capture_features_pr2.py](/capture_features_pr2) script and the model was trained and evaluated using the modified [train_svm_pr2.py](/train_svm_pr2.py) script. The scoring of the model generated the following confusion matrix for the cataloged objects displaying a fairly robust model:
  
  ![confusion matrix][image8]

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

The pipeline shown above was implemented in the [project_template.py](/pr2_robot/scripts/project_template.py) script which subscribes to the RGB-D datastream, pulling in raw point cloud data, pumpes it through the processing pipeline, publishes the processed data back to the ROS server, and packages a request to the pick_place_routine for each identified object. Upon receiving the request the pick_place_routine the PR2 robot has the information it needs to identify and locate each object in its pick list.

The PR2 robot was spawned in 3 different 'worlds', each with a different assortment of objects that it must identify and bin according to its pick list.

This implementation of the [project_template.py](/pr2_robot/scripts/project_template.py) script with the SVM model created above results in near 100% accuracy for all 'worlds' that it was run in.

The output_X.yaml files generated for each world can be seen at the root of this repo verifying the validity of this script.

![object recognition][image7]
![pick_place_routine][image9]


