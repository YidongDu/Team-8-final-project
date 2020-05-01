function pose = getLast3Dpose(values, marginals)
%getAll3Dposes get the 3Dpose location and covariance from the result
%   poses.cord{i}     saves the coordinate of the ith point
%   poses.cov{i}      saves the covariance of the ith point
import gtsam.*
keys = KeyVector(values.keys);
% Plot points and covariance matrices

for i = keys.size-1
    key = keys.at(i);
    p = values.at(key);
    if isa(p, 'gtsam.Pose3')
            pose.cov = marginals.marginalCovariance(key);
            pose.p = p.translation().vector();
            pose.R = p.rotation().matrix();
    end
end

end

