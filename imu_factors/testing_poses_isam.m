import gtsam.*

filename = 'rawdog/2011_09_30_drive_0018_sync/';
oxts = loadOxtsliteData(filename);
pose_matrices = convertOxtsToPose(oxts);

poseVarNames = {'i','pose'};
poseVarTypes = {'int32','gtsam.Pose3'};

PoseTable = [];
initial = Values;
    
for i = 1:length(pose_matrices)
    pose_new = table('Size',[1,length(poseVarTypes)],'VariableNames',poseVarNames,'VariableTypes',poseVarTypes);
    pose = Pose3(cell2mat(pose_matrices(i)));
    pose_new.i = i;
    pose_new.pose = pose;
    initial.insert(i-1,pose);
    PoseTable = [PoseTable; pose_new];
end

EdgeTable = [];

for i = 2:height(PoseTable)
    p0 = PoseTable.pose(i-1);
    p1 = PoseTable.pose(i);
    delta = p0.between(p1);
    EdgeTable = [EdgeTable;delta];
end

fg = NonlinearFactorGraph;
priorNoise = noiseModel.Diagonal.Sigmas([0.3;0.3;0.3;0.3;0.3;0.3]);
fg.add(PriorFactorPose3(0,Pose3(PoseTable.pose(1)),priorNoise));


for i = 1:length(EdgeTable)
    fg.add(BetweenFactorPose3(i-1,i, EdgeTable(i), priorNoise));
end

%% Plot Initial Estimate
cla
first = initial.at(0);
plot3(first.x(),first.y(),first.z(),'r*'); hold on
plot3DTrajectory(initial,'r-',false);
drawnow;

%% Solver object
isamParams = ISAM2Params;
isamParams.setFactorization('CHOLESKY');
isamParams.setRelinearizeSkip(10);
isam = gtsam.ISAM2(isamParams);
newFactors = NonlinearFactorGraph;
newValues = Values;


for measurementIndex = 1:height(PoseTable)
  
  if measurementIndex == 1
    %% Create initial estimate and prior on initial pose, velocity, and biases
    newValues.insert(PoseTable.i(1), PoseTable.pose(1));
    newFactors.add(PriorFactorPose3(0,PoseTable.pose(1),priorNoise));
    newFactors.add(PriorFactorPose3(1,PoseTable.pose(2),priorNoise));

  else
    % Add initial value
    if ~(measurementIndex > (2+2*5))
        newValues.insert(PoseTable.i(measurementIndex),PoseTable.pose(measurementIndex));
    end
    % Add Edges to Graph69
    if measurementIndex > (2+2*5)
        new_pose = currentPoseGlobal.compose(EdgeTable(mea))
        if((EdgeTable.j(EdgeIndex) == PoseTable.i(measurementIndex)) && (EdgeTable.i(EdgeIndex) == (PoseTable.i(measurementIndex)-1)))
            edge_point_o = Point3(EdgeTable.pose(EdgeIndex).Matrix);
            edge_rot_o = Rot3(quat2rotm([EdgeTable.rot(EdgeIndex).Matrix(4),EdgeTable.rot(EdgeIndex).Matrix(1),EdgeTable.rot(EdgeIndex).Matrix(2),EdgeTable.rot(EdgeIndex).Matrix(3)]));
            odometry = Pose3(edge_rot_o,edge_point_o);
            new_pose = currentPoseGlobal.compose(odometry);
            newValues.insert(PoseTable.i(measurementIndex),new_pose);
            EdgeTable.j(EdgeIndex);
            EdgeTable.i(EdgeIndex);
        end
        %edge_noise = noiseModel.Gaussian.SqrtInformation(chol(eye(6)));
        newFactors.add(BetweenFactorPose3(PoseTable.i(EdgeIndex),EdgeTable.j(EdgeIndex), edge_pose, edge_noise));
    end
    end
    
    % Update solver
    % =======================================================================
    % We accumulate 2*GPSskip GPS measurements before updating the solver at
    % first so that the heading becomes observable.
    if measurementIndex > (1 + 2*5)
      isam.update(newFactors, newValues);
      newFactors = NonlinearFactorGraph;
      newValues = Values;
      
      if rem(measurementIndex,5)==0 % plot every 10 time steps
        %cla;
%         plot3DTrajectory(isam.calculateEstimate, 'g-');
%         title('Estimated trajectory using ISAM2 (IMU+GPS)')
%         xlabel('[m]')
%         ylabel('[m]')
%         zlabel('[m]')
%         axis equal
%         drawnow;
%         view(3)
      end
      % =======================================================================
      results = isam.calculateEstimate();
      currentPoseGlobal = results.at(PoseTable.i(measurementIndex));
      newFactors.add(PriorFactorPose3(PoseTable.i(measurementIndex),currentPoseGlobal,priorNoise));
        
    end
  end
   
end % end main loop

axis equal
plot3DTrajectory(initial, 'r-')
plot3DTrajectory(isam.calculateEstimate, 'g-');
title('Estimated trajectory using ISAM2')

drawnow;
view(3)
disp('-- Reached end of sensor data')

