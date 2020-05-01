import gtsam.*

fsize = 16; % font size
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

%% Parse the data from the front end 
load('relative_pose.mat')
global steps
steps = 2700;
for j = 1:2761
    Transform{j}.R = Relative{j}(1:3,1:3);
    Transform{j}.p = Relative{j}(1:3,4);
end
np = length(Transform);

%% Read IMU data
%Read metadata and compute relative sensor pose transforms
%Read sensor metadata
IMU_metadata = importdata(findExampleDataFile('KittiEquivBiasedImu_metadata.txt'));
IMU_metadata = cell2struct(num2cell(IMU_metadata.data), IMU_metadata.colheaders, 2);
IMUinBody = Pose3.Expmap([IMU_metadata.BodyPtx; IMU_metadata.BodyPty; IMU_metadata.BodyPtz;
  IMU_metadata.BodyPrx; IMU_metadata.BodyPry; IMU_metadata.BodyPrz; ]);
if ~IMUinBody.equals(Pose3, 1e-5)
  error 'Currently only support IMUinBody is identity, i.e. IMU and body frame are the same';
end

%read data
filename = 'IMUdata';%filename = 'rawdog/2011_09_30_drive_0018_sync/';
oxts = loadOxtsliteData(filename);
pose_matrices = convertOxtsToPose(oxts);

timestamps = readtimestamps(strcat(filename,'/oxts/timestamps.txt'));

accx_index = 12;
accy_index = 13;
accz_index = 14;
wx_index = 18;
wy_index = 19;
wz_index = 20;

%% Gtsam setup
%solver objects
graph = NonlinearFactorGraph;
initials = Values();
isam2 = ISAM2();

%noise model
motion_noise = 0.002;
initModel = noiseModel.Diagonal.Sigmas([1e-3;1e-3;1e-3;1e-3;1e-3;1e-3]);
priorModel = noiseModel.Diagonal.Sigmas([0.15;0.15;0.15;0.15;0.15;0.15]);
odomModel = noiseModel.Diagonal.Sigmas([motion_noise;motion_noise; motion_noise; 1e-3; 1e-3; 1e-3]);
sensor_noise = 0.5;
obsModel = noiseModel.Diagonal.Sigmas(sensor_noise);
obsinitModel = noiseModel.Diagonal.Sigmas([0.5;0.5;2]);
%IMU data noise model
sigma_init_v = noiseModel.Isotropic.Sigma(3, 1000.0);
sigma_init_b = noiseModel.Isotropic.Sigmas([ 0.100; 0.100; 0.100; 5.00e-05; 5.00e-05; 5.00e-05 ]);
sigma_between_b = [ IMU_metadata.AccelerometerBiasSigma * ones(3,1); IMU_metadata.GyroscopeBiasSigma * ones(3,1) ];

% Initial pose
r = Rot3(eye(3));
t = Point3( [0;0;0]);
currentVelocityGlobal = LieVector([0;0;0]); % the vehicle is stationary at the beginning
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));
initials.insert(symbol('x', 0), Pose3(r, t));
initials.insert(symbol('v', 0), currentVelocityGlobal);
initials.insert(symbol('b', 0), currentBias);
graph.add(PriorFactorPose3(symbol('x', 0), Pose3(r, t), initModel));
graph.add(PriorFactorLieVector(symbol('v', 0), currentVelocityGlobal, sigma_init_v));
graph.add(PriorFactorConstantBias(symbol('b', 0), currentBias, sigma_init_b));

%other constants
g = [0;0;-9.8];
w_coriolis = [0;0;0];
integral_span = 5;

seen_landmarks = [];    % list of seen landmarks
landmark_class = [];
isam2.update(graph, initials)
results{1} = isam2.calculateBestEstimate();
graph_step{1} = isam2.getFactorsUnsafe();
maiginals = [];
marginals = Marginals(graph_step{1}, results{1});
last_pose = getLast3Dpose(results{1}, marginals);
% all_points = getAll3Dpoints(results{1}, marginals);

%% Main loop
index = steps;
counter = 1;

currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
currentBias, IMU_metadata.AccelerometerSigma.^2 * eye(3), ...
IMU_metadata.GyroscopeSigma.^2 * eye(3), IMU_metadata.IntegrationSigma.^2 * eye(3));
measCell = oxts(1);
measArray = measCell{1,1};
accMeas = [ measArray(accx_index); measArray(accy_index); measArray(accz_index)];
omegaMeas = [ measArray(wx_index); measArray(wy_index); measArray(wz_index)];
deltaT = timestamps(2)-timestamps(1);
currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);
for i = 1:index
    graph = NonlinearFactorGraph;
    initials = Values();
    r = Rot3(Transform{i}.R);
    t = Point3(Transform{i}.p);
    
    % visual odometry factor
    graph.add(BetweenFactorPose3(symbol('x', i-1), symbol('x', i), Pose3(r,t), odomModel));
    
    % add consecutive rigid body transformation result as initial pose
    r = Rot3(last_pose.R*Transform{i}.R);
    t = Point3(last_pose.R*Transform{i}.p+last_pose.p);
    if i>1
        graph.add(PriorFactorPose3(symbol('x', i-1), Pose3(Rot3(last_pose.R),Point3(last_pose.p)), priorModel));
    end
    initials.insert(symbol('x', i), Pose3(r, t));
    initials.insert(symbol('v', i), currentVelocityGlobal);
    initials.insert(symbol('b', i), currentBias);
    
    % IMU factor
    if counter == integral_span
        graph.add(ImuFactor( ...
        symbol('x', i-integral_span), symbol('v', i-integral_span), ...
        symbol('x', i), symbol('v', i), ...
        symbol('b', i), currentSummarizedMeasurement, g, w_coriolis));
        graph.add(BetweenFactorConstantBias(symbol('b', i-integral_span), symbol('b', i), imuBias.ConstantBias(zeros(3,1), zeros(3,1)), ...
        noiseModel.Diagonal.Sigmas(sqrt(integral_span) * sigma_between_b)));
        currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
        currentBias, IMU_metadata.AccelerometerSigma.^2 * eye(3), ...
        IMU_metadata.GyroscopeSigma.^2 * eye(3), IMU_metadata.IntegrationSigma.^2 * eye(3));
        counter = 0;
    end
    if i+1 < 2762
        measCell = oxts(i+1);
        measArray = measCell{1,1};
        accMeas = [ measArray(accx_index); measArray(accy_index); measArray(accz_index)];
        omegaMeas = [ measArray(wx_index); measArray(wy_index); measArray(wz_index)];
        deltaT = timestamps(i+2)-timestamps(i+1);
        currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);
        counter = counter + 1;
    end
    
    % optimize
    isam2.update(graph, initials)
    results{i+1} = isam2.calculateBestEstimate();
    currentVelocityGlobal = isam2.calculateEstimate(symbol('v', i));
    currentBias = isam2.calculateEstimate(symbol('b', i));
    graph_step{i+1} = isam2.getFactorsUnsafe();
    maiginals = [];
    marginals = Marginals(graph_step{i+1}, results{i+1});
    curr_pose = getLast3Dpose(results{i+1}, marginals);
    graph = NonlinearFactorGraph;
    initials = Values();
    last_pose = curr_pose;
    
    i
end
%
% video = VideoWriter('ISAM.mp4','MPEG-4')
% video.FrameRate = 2;
% open(video)
% writeVideo(video,F)
% close(video)



%% Plot results (VS ground truth trajectory)
import gtsam.*
% SLAM trajectory
graph_step{index}.print('\nFactor Graph:\n'); 
results{index}.print('Final Result: \n');
% Calculate marginal covariances for all poses
maiginals = [];
marginals = Marginals(graph_step{index}, results{index});

figure()
hold on; grid on; axis auto
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')

all_poses = getAll3Dposes(results{index}, marginals);
%plot3DTrajectory(results{index}, '', [], [], marginals);
%plot3DPoints(results{index}, [], marginals);
plot3DTrajectory(results{index}, '');
plot3DPoints(results{index});

axis equal



