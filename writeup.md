## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  See the example `output.yaml` for details on what the output should look like.  
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

[//]: # (Image References)

[image1]: ./misc_images/ex01-ransac-outliers.png
[image2]: ./misc_images/ex02-clusters.png
[image3]: ./misc_images/ex03-normalized-confusion.png
[image4]: ./misc_images/ex03-objects-recognized.png
[image5]: ./misc_images/test1_1.png
[image6]: ./misc_images/test1_2.png
[image7]: ./misc_images/test2_1.png
[image8]: ./misc_images/test2_2.png
[image9]: ./misc_images/test3_1.png
[image10]: ./misc_images/test3_2.png
[image11]: ./misc_images/proj_1.png
[image12]: ./misc_images/proj_2.png

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
After applying the RANSAC filter using a plane model and removing the inliers, the objects on top of the table were obtained, as shown in the following image.

![alt text][image1]

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  
Each object is separated using euclidean clustering, as shown in the following image.

![alt text][image2]

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
Training the support vector machine, the following normalized confusion matrix was obtained,

![alt text][image3]

The result object recognition is shown as follows,

![alt text][image4]

It can be noted that there are two disk_part labels, although the image only shows one. This is because the segmentation process groups the edge of the table as a cluster, as it is shown in the image of exercise 2. Hence, the object recognition process identifies the table edge as a disk_part. This issue is corrected in the project.


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

##### World 1
For the first world, the object recognition worked 100%, as it is shown in the following images:

![alt text][image5]
![alt text][image6]

The output_1.yaml file can be found in the pr2_robot/scripts directory.

##### World 2
In the second world, the perception recognized correctly 100% of the objects also, as it is shown in the following images:


![alt text][image7]
![alt text][image8]

The output_2.yaml file can be found in the pr2_robot/scripts directory.

##### World 3
In the third world, the support vector machine identified correctly 7 out 8 objects. The problem, is that the glue is behind the book, and this creates two problems:

1) The camera sees a the glue divided by the book, i.e., the camera captures a right portion of the glue and a left portion of it.

2) The clustering function creates two clusters for the glue (the left and right portions), so the support vector machine fails. It identifies the left portion as a sticky_notes, and the right portion as biscuits.

These problems can be seen in the following images:

![alt text][image9]
![alt text][image10]

Such problems origin in the clustering step.  In the following image, it can be seen that the glue is the object in white, but on the right there is a little pink colored object, which is part of the glue:

![alt text][image11]

In the next image, it can be appreciated that the book does not allow the camera to have a complete capture of the glue, but a divided one. This turns out in two separate portions of the glue:

![alt text][image12]

Hence, the support vector machine cannot identify the glue. The left portion is not complete to be identified as a glue, and the right portion is too small, and the output of the recognition algorithm is wrong.

From this case, it can be noted that this algorithm works as long as the objects can be clearly seen by the RGBD camera. If they are behind other objects, and only a portion of them can be seen, then the recognition step fails.

Another case where this perception fails is if the objects are touching each other. In that case, the euclidean clustering fails, since the criteria to associate a point in a group is distance. Objects that touch each other have their points close enough, so that the euclidean clustering step groups them as one object. A possible solution might be to group objects by more than one criteria, e.g., texture, color, euclidean distance, among others.

Finally, to achieve these results, it was not enough a pass through filter in the z direction. Since there are two baskets, part of them was captured by the camera, and the clustering step grouped them as separate objects.  Then the support vector machine recognized them as objects that were used to train it. To sove this issue, a pass through filter in the y direction was added, to keep the object just on top of the table, and to remove the portion of the baskets that were captured by accident.

The output_3.yaml file can be found in the pr2_robot/scripts directory.
