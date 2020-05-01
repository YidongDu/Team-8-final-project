import Lie.*
load('relative_pose.mat')

%% Get ground true and estimated pose twist
true_SE3 = {};
VIS_SE3 = {};
VIS_SE3{1}=eye(4);
vodo_SE3 = {};
vodo_SE3{1}=eye(4);
for i = 1:index+1
    true_SE3{i} = [gt{i}.R,gt{i}.p;[0,0,0,1]];
    if i>1
        VIS_SE3{i} = [all_poses.R{i-1},all_poses.p{i-1};[0,0,0,1]];
        vodo_SE3{i} = vodo_SE3{i-1} * Relative{i-1};
    end
end

%% Compute error
VIS_error = zeros(1,index+1);
vodo_error = zeros(1,index+1);
for i = 1:index+1
    VIS_error(i) = (norm(Log_SE3(VIS_SE3{i})-Log_SE3(true_SE3{i})))^2;
    vodo_error(i) = (norm(Log_SE3(vodo_SE3{i})-Log_SE3(true_SE3{i})))^2;
end

VIS_MSE=sum(VIS_error)/(index+1)
vodo_MSE=sum(vodo_error)/(index+1)

error = zeros(1,index+1);
for i = 2:index+1
    error(i) = (norm(gt{i}.p-all_poses.p{i-1}))^2;
end
pe=sum(error)/(index+1)