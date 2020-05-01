% Nearest Neighbor Data Association
function Mahalanobis = Maha(z,landmark_pose,landmark_cov,sensor_pose,sensor_cov,Q)
State_Covariance = blkdiag(landmark_cov,sensor_cov);
delta = [landmark_pose(1)-sensor_pose(1);...
        landmark_pose(2)-sensor_pose(2);...
        landmark_pose(3)-sensor_pose(3)];
q = delta'*delta;
z_hat = q^(1/2);
H1 = [-delta(1,1)/(q^(1/2)),-delta(2,1)/(q^(1/2)),...
      -delta(3,1)/(q^(1/2))];
H2 = [delta(1,1)/(q^(1/2)),delta(2,1)/(q^(1/2)),...
      delta(3,1)/(q^(1/2))];
H = [H1,H2];
S = ((H*State_Covariance*H'+Q)^(-1));
Mahalanobis = (z-z_hat)'*S*(z-z_hat);

