Unzip the IMUdata and observations zips first. Or you could use your own IMU data and landmark detection measurement data (as observations). The IMU data here is in KITTI IMU data formation. To know more, check http://www.cvlibs.net/datasets/kitti/raw_data.php. The landmark detection measurement data (observations) is organised as following: 
x_center_of_box,  y_center_of_box,  width_of_box,  height_of_box, class_probabilty_of_detected_class, class_number, [next 8 columns contains probability of the detection belonging to the different classes], relative_world_coord_x,  relative_world_coord_y ,  relative_world_coord_z

The ground truth trajectory is in true_pose.m. The visiual odometry SE(3) measurement is in relative_pose.m. They are obtained from front-end workds.

Then run ISAM2_back_end_main.m to solve the optimization problem and get the plot of SLAM results (the estimated robot poses and landmark poses).
