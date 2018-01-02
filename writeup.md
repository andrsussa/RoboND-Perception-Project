## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---
[//]: # (Image References)

[image1]: ./misc/FIlTER.PNG
[image2]: ./misc/CLUSTER.PNG
[image3]: ./misc/CONF2.PNG
[image4]: ./misc/CONF1.PNG
[image5]: ./misc/LABELS1.PNG
[image6]: ./misc/LABELS2.PNG
[image7]: ./misc/LABELS3.PNG

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

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

Filtering and RANSAC were implemented in the pcl_callback() function. This can be found in the file "project_template.py" in the path: pr2_robot/scripts/project_template.py

The code for the function can be seen below.

```python
# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()

    outlier_filter.set_mean_k(20)

    x = 0.1

    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_filtered_inlier = outlier_filter.filter()

    # outlier_filter.set_negative(True)
    # cloud_filtered_outlier = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
    vox = cloud_filtered_inlier.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    passthrough_z = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'

    passthrough_z.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 0.9

    passthrough_z.set_filter_limits(axis_min, axis_max)
    cloud_filtered_pass = passthrough_z.filter()

    passthrough_y = cloud_filtered_pass.make_passthrough_filter()
    filter_axis = 'y'

    passthrough_y.set_filter_field_name(filter_axis)
    axis_min = -0.4
    axis_max = 0.4

    passthrough_y.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough_y.filter()

    # TODO: RANSAC Plane Segmentation
    segmenter = cloud_filtered.make_segmenter()
    segmenter.set_model_type(pcl.SACMODEL_PLANE)
    segmenter.set_method_type(pcl.SAC_RANSAC)

    max_distance = 0.01
    segmenter.set_distance_threshold(max_distance)

    inliers, coefficients = segmenter.segment()

    # TODO: Extract inliers and outliers
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    cloud_table = cloud_filtered.extract(inliers, negative=False)
```
In it, it can be seen the pipeline steps for:

1. Statistical outlier filtering
2. Downsampling
3. Passthrough filter x 2
4. RANSAC plane segmentation

The result of the filtering can be seen below.

![alt text][image1]


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

Clustering for segmentation was implemented in the pcl_callback() function after filtering. This can be found in the file "project_template.py" in the path: pr2_robot/scripts/project_template.py

The code for the function can be seen below.

```python
    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(2500)

    ec.set_SearchMethod(tree)

    cluster_indices = ec.Extract()

    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0], white_cloud[indice][1], white_cloud[indice][2], rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_objects = pcl_to_ros(cloud_objects)
    ros_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_objects)
    pcl_table_pub.publish(ros_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

```

The result of the clustering can be seen below.

![alt text][image2]

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

Feature extraction and SVM training was done through scripts in another package called "sensor_stick". The files relevant to the submission of this project were included in a folder called "sensor_stick_aux_scripts" in this path: pr2_robot/scripts/sensor_stick_aux_scripts

The code that performs the prediction based on the obtained model is seen below.

The files of the models used in the three scenarios were also stored in the script folder of the project.

```python
# Exercise-3 TODOs:
    detected_objects_labels = []
    detected_objects = []

    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        ros_cluster_object = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster_object, using_hsv=True)
        normals = get_normals(ros_cluster_object)
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
        do.cloud = ros_cluster_object
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)
```

These are the resulted confusion matrix when generating the model for scenario 3.

![alt text][image3]


![alt text][image4]


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

The yaml files related to each of the scenarios are stored in the scripts folder of this project and the are called output_1.yaml, output_2.yaml and output_3.yaml, for each scenario respectively.

**Scenario 1**

These are the labels predicted for this scenario.

![alt text][image5]

**Scenario 2**

These are the labels predicted for this scenario.

![alt text][image6]

**Scenario 3**

These are the labels predicted for this scenario.

![alt text][image7]

Regarding the filtering stage, the most challenging part was finding the parameteres for the statistical outlier filtering because I tried to find proper values at the begining of the implementation, without knowing if it was influencing positively the final clustered output. Later, when watching the output of the clusters, I started to notice where the noise was really affecting the system. It was then when I properly tested different parameters for this outlier filtering and settled on the values showed in the code.

Regarding the passthrough filtering, it was different from exercises in the course in the sense that this time it needed two filters rather than one. One on the Z axis and the other on the Y axis. It was just a matter of trial and error to finally find proper boundaries for the filter as seen in the code.

In the case of RANSAC and downsampling, the same configuration as previous exercises fitted well for this case.

Talking about the "pr2_mover" function, my strategy was to iterate over the pick list given, and determine, one by one, if the object in the list was detected or not in the scene. To ease this process, I decided to add another passing parameter to this function, which was detected labels. This way I could ask if the object label in the list was present, and if it was I could get its index. Then, having the index I could retrieve the cloud of the object and find its centroid. In the case that
the object was not found, the loop would just continue to the next object.

The rest of the function was pretty straight forward, in the sense that populating all the parameters for the yaml file and the service was clear and easily obtainable.

For scenario 3, there was an object that could not be detected. It was clear that this happened because the object was being blocked by another, hence it was not on the range of sight of the sensor. The way this could be improved is to run the detection pipeline again each time an object is stored in a box, although this would modify in great matter the current structure of the system. Another option would be to run the detection again, only in the case an object of the list is not found in
the detected objects, only expecting that the one of the objects that were already retrieved was the one blocking the sight. Clearly, this would not work otherwise, or in the case the that object in question is the first on the list. Finally, another solution would be to obtain an additional sensor, which has to be analyzed deeply, since this would represent double the processing power or double the processing time, apart from more complex logic.

Finally, when I encountered problems in proper label prediction for the objects. I first identified if at least the correct amount of objects were detected, if not, it meant that the clustering was not working and statistical outlier filtering had to be tuned. If otherwise, another stage of training was performed or even extended.
