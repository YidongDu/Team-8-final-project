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

Odometry = [];

for i = 2:height(PoseTable)
    p0 = PoseTable.pose(i-1);
    p1 = PoseTable.pose(i);
    delta = p0.between(p1);
    Odometry = [Odometry;delta];
end

fg = NonlinearFactorGraph;
priorNoise = noiseModel.Diagonal.Sigmas([0.3;0.3;0.3;0.3;0.3;0.3]);
fg.add(PriorFactorPose3(0,Pose3(PoseTable.pose(1)),priorNoise));


for i = 1:length(Odometry)
    fg.add(BetweenFactorPose3(i-1,i, Odometry(i), priorNoise));
end

%% Plot Initial Estimate
cla
first = initial.at(0);
plot3(first.x(),first.y(),first.z(),'r*'); hold on
plot3DTrajectory(initial,'r-',false);
drawnow;

%% Read again, now with all constraints, and optimize
optimizer = GaussNewtonOptimizer(fg, initial);
result = optimizer.optimizeSafely();
plot3DTrajectory(result, 'g-', false); %axis equal;

view(3); axis equal;