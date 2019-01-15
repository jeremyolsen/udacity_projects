# Udacity Perception Project
**Jeremy Olsen - slack: jolsen-oas**

[//]: #parameterseferences
[Ex1_image1]: ./img/Voxel_downsampled.png
[Ex1_image2]: ./img/passthrough_filter.png
[Ex1_image3]: ./img/RANSAC_inliers_table_only.png
[Ex1_image4]: ./img/RANSAC_objects_only.png
[Ex2_image1]: ./img/published_pcl_table.png
[Ex2_image2]: ./img/published_pcl_objects.png
[Ex2_image3]: ./img/published_pcl_cluster.png
[Ex3_image1]: ./img/confusion_matricies_for_train_svm.png
[Ex3_image2]: ./img/labeled_pcl_objects.png
[Project_image1]: ./img/world_1.png
[Project_image2]: ./img/world_2.png
[Project_image3]: ./img/world_3.png

## Exercise 1 - Filtering and Segmentation

Covered implementation of the Voxel Downsample, Passthrough Filter, and RANSAC Plane fitting techniques for our perception pipeline.

- A Voxel Downsample filter was used to create a point cloud of the table and all of the objects in the scene
- Implemented a Passthrough filter was used to 'crop' the scene on a desired axis.  In this case we only used the 'z' axis to remove the table floor and legs, leaving only the tabletop and the objects
- A RANSAC plane fitting filter was used used to find the objects vs. the table top.  The filter was setup to match the point cloud indicies to a predefined plane model.  The negative result of that filter was used to obtain the objects only.

All Examples of the above descriptions are shown below

**Voxel Downsample Example**
![Voxel Downsampled][Ex1_image1]

**Passthrough Filter Example**
![Passthrough Filter][Ex1_image2]

**RANSAC Table Only Example**
![RANSAC Table Only][Ex1_image3]

**RANSAC Objects Only Example**
![RANSAC Objects Only][Ex1_image4]

## Exercise 2 - Clustering for Segmentation

Covered implementing further pipeline techniques for clustering and segmentation.  The focus was upon applying Euclidean clustering to create separate clusters for individual items.

**PCL Table Only Example**
![PCL Table Only][Ex2_image1]

**PCL Objects Example**
![PCL Objects][Ex2_image2]

**PCL Cluster Example**
![PCL Cluster][Ex2_image3]

## Exercise 3 - Object Recognition

Covered implementing histograms for that would be utilized by Support Vector Machine which is a supervised machine learning algorithm.  Two new processes of capturing features and then training the SVM to look for the objects were utilized.  The output of a confusion matrix showed how accurate the SVM was in identifying the objects.  Lastly, the prediction portion of the SVM was implemented in the pipeline and labels were then added to the objects in our original scene.

**Confusion Matrices for SVM Training**
![Confusion Matrices for SVM Training][Ex3_image1]

**Labeled PCL Objects**
![Labeled PCL Objects][Ex3_image2]

##Perception Project - Setup for Pick-n-Place

I was able to tune my pipeline to get 100% prediction/labeling results for each of the required worlds.  YAML files are also included with the submission as required.

**World 1 Labeled Objects**
![World 1 Labeled Objects][Project_image1]

**World 2 Labeled Objects**
![World 2 Labeled Objects][Project_image2]

**World 3 Labeled Objects**
![World 3 Labeled Objects][Project_image3]

##Challenges

I went ahead and coded for the the challenge requirements, however I discovered that there is a huge amount of tuning that I would need to perform to get each portion working perfectly and opted to not finish as I'm feeling that I'm running out of time.  Here are the steps I did perform:
 - Completed the contract to publish to the collision service including filtering out the only object in the PCL that was going to be picked up
 - Completed the contract to publish to the pick-n-place service.
 - Completed sending an angle to the 'world_joint_controller' to rotate the base to find all of the collision objects.
 - Added in the feature to rotate the robot back and forth based upon it's current angle as this was needed to tune for the tabletop challenge portion.
 
 I'm also submitting a GIF of the tabletop challenge portion that is partially complete.  As I stated above, the time to finish tuning is getting out of hand and I need to move on to the next project.  Any suggestions you might have on how to make this work better would be appreciated. 