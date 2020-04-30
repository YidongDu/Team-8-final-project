clear; clc;
% Load groundtruth data
filename = '/Users/rogermei/Desktop/Umich 20 Winter/EECS_568/Final_project/visual_odometry/';
oxts = loadOxtsliteData(filename);
pose_matrices = convertOxtsToPose(oxts);
length(pose_matrices)
for i = 2 : length(pose_matrices)
    Relative{i-1} = pose_matrices{i-1}^(-1)*pose_matrices{i};
end