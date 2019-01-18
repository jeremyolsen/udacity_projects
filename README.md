# Udacity Nanodegree Projects  

## Robotics Software Engineer Nanodegree

### Term 1 Projects 
* **Search Map Project (Mars Rover Simulation)**
  
  This was the very first project of the ND which served as an introduction to the course.  After reviewing the lesson materials, the     goals of this project were to write code to autonomously map a simulated environment and search for samples of interest. 
  
  The first step was to get a Unity simulator up and running. Once the simulator was running, we were to record data in training mode test functionality of the project code in the Jupyter notebook.  Were were to add custom code to search for obstacles using techniques with OpenCV.  We were then create a series of decision making commands to help navigate the rover around the map and pick up 6 randomly placed gold colored rocks located througout the map as an optional goal.

* **Robotic Arm: Pick N Place**

  The course material focused upon teaching forward and inverse kinematics.  The project expanded upon this knowlege by providing us a Gazebo simulated environment to peferform inverse kinematics when provided a list of end-effector poses and calculate joint angles for the Kuka KR210.

* **Perception**

  Must successfully complete a tabletop pick and place operation using a simulated PR2 robot. The PR2 has been outfitted with an RGB-D sensor much like the one you used in previous exercises. This sensor however is a bit noisy much like real sensors.

  Given a cluttered tabletop scenario, you must implement a perception pipeline to identify target objects from a so-called “Pick-List”.  Using a particular order, pick up those objects and place them in corresponding dropboxes located in the environment.

* **Deep Learning Project**

  Build a segmentation network, based upon Tensor Flow framework, that will be used to create a model from images that are captured via as simulator environment.  This model is later used to follow a specific object around the simulated environment.  Next is training and testing of the model.  If goals are not achieved, then capture and further augment the dataset.


### Term 2 Projects
* **Inference Project**

  Create a custom classification network Using Nvidia DIGITS workspace after collecting our own images for a dataset.  This required setting up the data set using the KITTI structure.  Optionally, deploy the newly trained model to the Nvidia TX2.

* **Localization Project**

  Using ROS, Gazebo, and Rviz to accurately localize a mobile robot inside a provided map.  The requirements are to first build a robot in Gazebo that uses a laser range finder and wheel encoders to use for the localization application.  Create a ROS package that launches the custom robot model in a Gazebo world and utilizes packages like AMCL and the Navigation Stack.  Once the complete simulation is setup, tuning and adding specific parameters corresponding to each package is required to achieve the best localization results.

* **Reinforced Learning Project**

  Goal of the project is to create a DQN agent and define reward functions to teach a robotic arm to carry out two primary objectives.  First, have any part of the robot arm touch the object of interest with at least 90% accuracy.  Second, have only a small part of the gripper touch the object with at least 80% accuracy.  The project is all written in C++ and is controlling Gazebo via the external API.  The DQN agent is utilizing Torch.

* **SLAM Project**

  Develop a ROS package to interface with the rtabmap_ros package, including all the correct topic links are setup correctly. Modify the custom Gazebo model from the previous project by adding an RGB-D camera which will be controlled by the ROS teleop package. Goal is to create a custom environment and map it entirely using the camera enabled custom bot.  Extensive debugging is performed using roswtf and rqt common plugins.

* **Home Service Robot**

  Final project for the RoboND.  Goal is to utilized many of the skills obtained from the ND and setup the ROS/Gazebo environment from scratch.  Part of the task is to create a customized Gazebo building using the editor in which the simulated robot will interact with.  Setup a custom ROS package that will contain  that will interact with the gmapping, AMCL,  turtlebot_teleop, turtlebot_rviz and turtlebot_gazebo packages.  The end goal is to reach multiple goals around the environment using localization and mapping to perform SLAM. 


## Computer Vision Nanodegree
* **Facial Keypoint Project**

  This project uses computer vision techniques and deep learning architectures to build a facial keypoint detection system. Facial keypoints include points around the eyes, nose, and mouth on a face and are used in many applications. The completed code should be able to look at any image, detect faces, and predict the locations of facial keypoints on each face.  The majority of the project focuses upon defining and training of a CNN to predict the facial keypoints.

* **Image Captioning using Machine Learning Project**

  Create a neural network architecture to automatically generate captions from images.  The network will be trained using the MS COCO dataset.  All inputs are passed as a sequence into an LSTM, which processes these sequentially to derive a series of word to describe the image.

* **SLAM Project**

  Implement SLAM (Simultaneous Localization and Mapping) for a 2 dimensional world.  This will combine sensor measurements and movement to create a simple map of the environment.

