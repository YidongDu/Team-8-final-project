function poses = getAll3Dposes(values, marginals)
%getAll3Dposes get the 3Dpose location and covariance from the result
%   poses.cord{i}     saves the coordinate of the ith point
%   poses.cov{i}      saves the covariance of the ith point
import gtsam.*
keys = KeyVector(values.keys);
% Plot points and covariance matrices
index = 1;
for i = 0:keys.size-1
    key = keys.at(i);
    p = values.at(key);
    if isa(p, 'gtsam.Pose3')
            poses.cov{index} = marginals.marginalCovariance(key);
            poses.p{index} = p.translation().vector();
            poses.R{index} = p.rotation().matrix();
            index = index + 1;
    end
end

end

