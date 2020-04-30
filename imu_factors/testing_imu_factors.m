import gtsam.*
addpath('');

%% Read metadata and compute relative sensor pose transforms
% IMU metadata
disp('-- Reading sensor metadata')
IMU_metadata = importdata(findExampleDataFile('KittiEquivBiasedImu_metadata.txt'));
IMU_metadata = cell2struct(num2cell(IMU_metadata.data), IMU_metadata.colheaders, 2);
IMUinBody = Pose3.Expmap([IMU_metadata.BodyPtx; IMU_metadata.BodyPty; IMU_metadata.BodyPtz;
  IMU_metadata.BodyPrx; IMU_metadata.BodyPry; IMU_metadata.BodyPrz; ]);
if ~IMUinBody.equals(Pose3, 1e-5)
  error 'Currently only support IMUinBody is identity, i.e. IMU and body frame are the same';
end

%% Read Data

filename = 'rawdog/2011_09_30_drive_0018_sync/';
oxts = loadOxtsliteData(filename);
pose_matrices = convertOxtsToPose(oxts);

timestamps = readtimestamps(strcat(filename,'/oxts/timestamps.txt'));

accx_index = 12;
accy_index = 13;
accz_index = 14;
wx_index = 18;
wy_index = 19;
wz_index = 20;

%% Get initial conditions for the estimated trajectory
currentPoseGlobal = Pose3(Rot3(eye(3)), Point3(0,0,0)); % initial pose is the reference frame (navigation frame)
currentVelocityGlobal = LieVector([0;0;0]); % the vehicle is stationary at the beginning
currentBias = imuBias.ConstantBias(zeros(3,1), zeros(3,1));
sigma_init_x = noiseModel.Isotropic.Precisions([ 0.0; 0.0; 0.0; 1; 1; 1 ]);
noise_test = noiseModel.Diagonal.Sigmas([0.1;0.1;0.1;0.1;0.1;0.1]);
sigma_init_v = noiseModel.Isotropic.Sigma(3, 1000.0);
sigma_init_b = noiseModel.Isotropic.Sigmas([ 0.100; 0.100; 0.100; 5.00e-05; 5.00e-05; 5.00e-05 ]);
sigma_between_b = [ IMU_metadata.AccelerometerBiasSigma * ones(3,1); IMU_metadata.GyroscopeBiasSigma * ones(3,1) ];
g = [0;0;-9.8];
w_coriolis = [0;0;0];

%% Solver object
isamParams = ISAM2Params;
isamParams.setFactorization('CHOLESKY');
isamParams.setRelinearizeSkip(10);
isam = gtsam.ISAM2(isamParams);
newFactors = NonlinearFactorGraph;
newValues = Values;

numIndices = 10;
%% Main Loop

for measurementIndex = 1:(length(oxts)/numIndices)
    % At each non=IMU measurement we initialize a new node in the graph
      currentPoseKey = symbol('x',measurementIndex);
      currentVelKey =  symbol('v',measurementIndex);
      currentBiasKey = symbol('b',measurementIndex);

      if measurementIndex == 1
        %% Create initial estimate and prior on initial pose, velocity, and biases
        newValues.insert(currentPoseKey, currentPoseGlobal);
        newValues.insert(currentVelKey, currentVelocityGlobal);
        newValues.insert(currentBiasKey, currentBias);
        newFactors.add(PriorFactorPose3(currentPoseKey, currentPoseGlobal, noise_test));
        newFactors.add(PriorFactorLieVector(currentVelKey, currentVelocityGlobal, sigma_init_v));
        newFactors.add(PriorFactorConstantBias(currentBiasKey, currentBias, sigma_init_b));
      else
          
          IMUindices = (numIndices*(measurementIndex-1)+1):(numIndices*measurementIndex); %replace once visual odometry is working
          currentSummarizedMeasurement = gtsam.ImuFactorPreintegratedMeasurements( ...
          currentBias, IMU_metadata.AccelerometerSigma.^2 * eye(3), ...
          numIndices^2*IMU_metadata.GyroscopeSigma.^2 * eye(3), IMU_metadata.IntegrationSigma.^2 * eye(3));

        for imuIndex = IMUindices
          measCell = oxts(imuIndex);
          measArray = measCell{1,1};
          accMeas = [ measArray(accx_index); measArray(accy_index); measArray(accz_index)];
          omegaMeas = [ measArray(wx_index); measArray(wy_index); measArray(wz_index)];
          deltaT = timestamps(imuIndex)-timestamps(imuIndex-1);
          currentSummarizedMeasurement.integrateMeasurement(accMeas, omegaMeas, deltaT);
        end

        % Create IMU factor
         newFactors.add(ImuFactor( ...
           currentPoseKey-1, currentVelKey-1, ...
           currentPoseKey, currentVelKey, ...
           currentBiasKey, currentSummarizedMeasurement, g, w_coriolis));
        %odomp1 = Pose3(pose_matrices{1,5*measurementIndex});
        %odomp0 = Pose3(pose_matrices{1,5*(measurementIndex-1)});
        %odom = odomp1.between(odomp0);
        %newFactors.add(BetweenFactorPose3(currentPoseKey-1,currentPoseKey, odom, noise_test));

        newFactors.add(BetweenFactorConstantBias(currentBiasKey-1, currentBiasKey, imuBias.ConstantBias(zeros(3,1), zeros(3,1)), ...
        noiseModel.Diagonal.Sigmas(sqrt(numel(IMUindices)) * sigma_between_b)));
        %Add poses
        odom_string = evalc('disp(currentSummarizedMeasurement)');
        [deltaPij,deltaVij,deltaRij] = extractfromprint(odom_string);
        odom = Pose3(Rot3(deltaRij),Point3(deltaVij(1),deltaVij(2),deltaVij(3)));
        
        newpose = currentPoseGlobal.compose(odom);
        newValues.insert(currentPoseKey, newpose);
        
        %if mod(measurementIndex, 10) == 0
        %  newFactors.add(PriorFactorPose3(currentPoseKey, Pose3(pose_matrices{1,5*measurementIndex}), noise_test));
        %end
        
        %newValues.insert(currentPoseKey, Pose3(pose_matrices{1,measurementIndex}));
        newValues.insert(currentVelKey, currentVelocityGlobal);
        newValues.insert(currentBiasKey, currentBias);

        % Update solver
        % =======================================================================
        % We accumulate 2*GPSskip GPS measurements before updating the solver at
        % first so that the heading becomes observable.
        if measurementIndex > 10
          isam.update(newFactors, newValues);
          newFactors = NonlinearFactorGraph;
          newValues = Values;

          if rem(measurementIndex,10)==0 % plot every 10 time steps
            cla;
            plot3DTrajectory(isam.calculateEstimate, 'g-');
            title('Estimated trajectory using ISAM2 (IMU+GPS)')
            xlabel('[m]')
            ylabel('[m]')
            zlabel('[m]')
            axis equal
            drawnow;
          end
          % =======================================================================
          currentPoseGlobal = isam.calculateEstimate(currentPoseKey);
          currentVelocityGlobal = isam.calculateEstimate(currentVelKey);
          currentBias = isam.calculateEstimate(currentBiasKey);
          newFactors.add(PriorFactorPose3(currentPoseKey,currentPoseGlobal,noise_test));

        end
      end
end
