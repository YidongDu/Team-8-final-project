clear;clc;
%% Monocular Visual Odometry
% Visual odometry is the process of determining the location and orientation
% of a camera by analyzing a sequence of images. Visual odometry is used in
% a variety of applications, such as mobile robots, self-driving cars, and 
% unmanned aerial vehicles. This example shows you how to estimate the
% trajectory of a single calibrated camera from a sequence of images. 

%% Initialization
% Camera intrinsic matrix
% To access its format: http://ksimek.github.io/2013/08/13/intrinsic/
K = [707 0 602; 0 707 183; 0 0 1]';
cameraParams = cameraParameters('IntrinsicMatrix', K);
% Store the file into images
images = imageDatastore('/Users/rogermei/Desktop/Umich 20 Winter/EECS_568/Final_project/visual_odometry/image_02/');
% Load groundtruth data
filename = '/Users/rogermei/Desktop/Umich 20 Winter/EECS_568/Final_project/visual_odometry/';
oxts = loadOxtsliteData(filename);
pose_matrices = convertOxtsToPose(oxts);
groundTruthPoses = get_pose_from_posemat(pose_matrices);
% Create an empty imageviewset object to manage the data associated with each view.
vSet = viewSet;
% Read and display the first image.
Irgb = readimage(images, 1);
player = vision.VideoPlayer('Position', [20, 400, 650, 510]);
step(player, Irgb);
%%
% Convert to gray scale and undistort. In this example, undistortion has no 
% effect, because the images are synthetic, with no lens distortion. However, 
% for real images, undistortion is necessary.

prevI = undistortImage(rgb2gray(Irgb), cameraParams); 

% Detect features. 
prevPoints = detectSURFFeatures(prevI,'MetricThreshold', 6000);

% Extract features. Using 'Upright' features improves matching quality if 
% the camera motion involves little or no in-plane rotation.
prevFeatures = extractFeatures(prevI, prevPoints, 'Upright', true);


% Define a transfer function which transfer estimated pose to actual pose
% Rotate around y for -90 --> Rotate around x for -90
R1 = [cos(pi/2),0,sin(pi/2);
      0,1,0;
      -sin(pi/2),0,cos(pi/2)];
R2 = [1,0,0;
      0,cos(-pi/2),-sin(-pi/2);
      0,sin(-pi/2),cos(-pi/2)];
T = R2*R1;

% Add the first view. Place the camera associated with the first view
% at the origin, oriented along the Z-axis.
% R1 = [cos(-pi/2),0,sin(-pi/2);
%       0,1,0;
%       -sin(-pi/2),0,cos(-pi/2)];
viewId = 1;
vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', eye(3),...
    'Location', [0 0 0]);
% vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', T*eye(3),...
%     'Location', [0 0 0]);

%% Plot Initial Camera Pose
% Create two graphical camera objects representing the estimated and the
% actual camera poses based on ground truth data.

% Setup axes.
figure
axis([-400, 300, -140, 20, -50, 300]);

% % Set Y-axis to be vertical pointing down.
% view(gca, 3);
% set(gca, 'CameraUpVector', [0, -1, 0]);
% camorbit(gca, -120, 0, 'data', [0, 1, 0]);

% Set Y-axis to be vertical pointing down.
view(gca, 3);
set(gca, 'CameraUpVector', [0, -1, 0]);
camorbit(gca, -120, 0, 'data', [0, 1, 0]);


grid on
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
hold on

% Plot estimated camera pose. 

cameraSize = 7;
camEstimated = plotCamera('Size', cameraSize, 'Location',...
    vSet.Views.Location{1}, 'Orientation', vSet.Views.Orientation{1},...
    'Color', 'g', 'Opacity', 0);
camActual = plotCamera('Size', cameraSize, 'Location', ...
    groundTruthPoses.Location{1}, 'Orientation', ...
    groundTruthPoses.Orientation{1}, 'Color', 'b', 'Opacity', 0);
% Initialize camera trajectories.
trajectoryEstimated = plot3(0, 0, 0, 'g-');
trajectoryActual    = plot3(0, 0, 0, 'b-');
legend('Estimated Trajectory', 'Actual Trajectory');
title('Camera Trajectory');
%% Estimate the Pose of the Second View
% Detect and extract features from the second view, and match them to the
% first view using <matlab:edit('helperDetectAndMatchFeatures.m') helperDetectAndMatchFeatures>. 
% Estimate the pose of the second view relative to the first view using 
% <matlab:edit('helperEstimateRelativePose.m') helperEstimateRelativePose>,
% and add it to the |viewSet|.
% Read and display the image.
viewId = 2;
Irgb = readimage(images, viewId);
step(player, Irgb);

% Convert to gray scale and undistort.
I = undistortImage(rgb2gray(Irgb), cameraParams);

% Match features between the previous and the current image.
[currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(...
    prevFeatures, I);

% Estimate the pose of the current view relative to the previous view.
[orient, loc, inlierIdx] = helperEstimateRelativePose(...
    prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)), cameraParams);
% Exclude epipolar outliers.
indexPairs = indexPairs(inlierIdx, :);
    
% Add the current view to the view set.
vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, ...
    'Location', loc);
% Store the point matches between the previous and the current views.
vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);
%%
% The location of the second view relative to the first view can only be
% recovered up to an unknown scale factor. Compute the scale factor from 
% the ground truth using <matlab:edit('helperNormalizeViewSet.m') helperNormalizeViewSet>,
% simulating an external sensor, which would be used in a typical monocular
% visual odometry system.

vSet = helperNormalizeViewSet(vSet, groundTruthPoses);

%%
% Update camera trajectory plots using 
% <matlab:edit('helperUpdateCameraPlots.m') helperUpdateCameraPlots> and
% <matlab:edit('helperUpdateCameraTrajectories.m') helperUpdateCameraTrajectories>.

helperUpdateCameraPlots(viewId, camEstimated, camActual, poses(vSet), ...
    groundTruthPoses);

helperUpdateCameraTrajectories(viewId, trajectoryEstimated, trajectoryActual,...
    poses(vSet), groundTruthPoses);

prevI = I;
prevFeatures = currFeatures;
prevPoints   = currPoints;

%% Bootstrap Estimating Camera Trajectory Using Global Bundle Adjustment
% Find 3D-to-2D correspondences between world points triangulated from the 
% previous two views and image points from the current view. Use 
% <matlab:edit('helperFindEpipolarInliers.m') helperFindEpipolarInliers> 
% to find the matches that satisfy the epipolar constraint, and then use
% <matlab:edit('helperFind3Dto2DCorrespondences.m') helperFind3Dto2DCorrespondences>
% to triangulate 3-D points from the previous two views and find the
% corresponding 2-D points in the current view.
%
% Compute the world camera pose for the current view by solving the 
% perspective-n-point (PnP) problem using |estimateWorldCameraPose|. For 
% the first 15 views, use global bundle adjustment to refine the entire
% trajectory. Using global bundle adjustment for a limited number of views
% bootstraps estimating the rest of the camera trajectory, and it is not
% prohibitively expensive.
i = 1;
for viewId = 3:numel(images.Files)
    viewId
    % Read and display the next image
    Irgb = readimage(images, viewId);
    step(player, Irgb);
    Q(i) = im2frame(Irgb);
    % Convert to gray scale and undistort.
    I = undistortImage(rgb2gray(Irgb), cameraParams);
    
    % Match points between the previous and the current image.
    [currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(...
        prevFeatures, I);
      
    % Eliminate outliers from feature matches.
    inlierIdx = helperFindEpipolarInliers(prevPoints(indexPairs(:,1)),...
        currPoints(indexPairs(:, 2)), cameraParams);
    indexPairs = indexPairs(inlierIdx, :);
    
    % Triangulate points from the previous two views, and find the 
    % corresponding points in the current view.
    [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(vSet,...
        cameraParams, indexPairs, currPoints);
    
    % Since RANSAC involves a stochastic process, it may sometimes not
    % reach the desired confidence level and exceed maximum number of
    % trials. Disable the warning when that happens since the outcomes are
    % still valid.
    warningstate = warning('off','vision:ransac:maxTrialsReached');
    
    % Estimate the world camera pose for the current view.
    [orient, loc] = estimateWorldCameraPose(imagePoints, worldPoints, ...
        cameraParams, 'Confidence', 99.99, 'MaxReprojectionError', 0.8);
    fprintf('Estimated Orientation')
    orient
    loc
    fprintf('Actual Orientation')
    groundTruthPoses.Orientation{1,viewId}
    groundTruthPoses.Location{1,viewId}
    % Restore the original warning state
    warning(warningstate)
    
    % Add the current view to the view set.
    vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, ...
        'Location', loc);
    
    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);    
    
    tracks = findTracks(vSet); % Find point tracks spanning multiple views.
        
    camPoses = poses(vSet);    % Get camera poses for all views.
    
    
    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
    
    % Refine camera poses using bundle adjustment.
    [~, camPoses] = bundleAdjustment(xyzPoints, tracks, camPoses, ...
        cameraParams, 'PointsUndistorted', true, 'AbsoluteTolerance', 1e-9,...
        'RelativeTolerance', 1e-9, 'MaxIterations', 300);
        
    vSet = updateView(vSet, camPoses); % Update view set.
    
    % Bundle adjustment can move the entire set of cameras. Normalize the
    % view set to place the first camera at the origin looking along the
    % Z-axes and adjust the scale to match that of the ground truth.
    vSet = helperNormalizeViewSet(vSet, groundTruthPoses);
    
    % Update camera trajectory plot.
    helperUpdateCameraPlots(viewId, camEstimated, camActual, poses(vSet), ...
        groundTruthPoses);
    helperUpdateCameraTrajectories(viewId, trajectoryEstimated, ...
        trajectoryActual, poses(vSet), groundTruthPoses);

    i = i + 1;
    prevI = I;
    prevFeatures = currFeatures;
    prevPoints   = currPoints;  

end

transform = [0 0 1 0;-1 0 0 0;0 -1 0 0;0 0 0 1];
% saver = [pose{starter}(1:3,1:3),saver(1:3,4);0 0 0 1];
for i = 1:height(camPoses)
    SE3{i} = transform*[camPoses.Orientation{i},camPoses.Location{i}' ;0 0 0 1]*(transform^-1);
%     SE3{i} = [camPoses.Orientation{i},camPoses.Location{i}' ;0 0 0 1];
    SE3_actual{i} = pose_matrices{i};
    if i>1
        Relative{i} = SE3{i-1}^(-1)*SE3{i};
        Relative_actual{i} = SE3_actual{i-1}^(-1)*SE3_actual{i};
    end
end

% SE3_total{counter} = SE3;
% Relative_total{counter} = Relative;
% j = [];
% p = [];
% counter2 = 1;
% for k = 1:length(SE3_total)
%     for q = 1:length(SE3)
%         j(1:3,counter2) = SE3_total{k}{q}(1:3,4);
%         p(1:3,counter2) = pose{counter2}(1:3,4);
%         counter2 = counter2 + 1;
%     end
% end 
% 
% saver = SE3{length(SE3)};
%% Plot of Ground Truth Pose and Pose Obtained From Monocular Visual Odometry
% figure
% k = norm(j,1);
% scatter3(j(1,:),j(2,:),j(3,:))
% hold on
% scatter3(p(1,:),p(2,:),p(3,:),'r')
% axis([-k, k, -k, k, -k, k])
% grid on
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');
% hold on
% counter = counter + 1;
