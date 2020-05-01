function points = getAll3Dpoints(values, marginals)
%getAll3Dpoints get the 3Dpoints(landmark) location and covariance from the result
%   points.cord{i}     saves the coordinate of the ith point
%   points.cov{i}      saves the covariance of the ith point
import gtsam.*
keys = KeyVector(values.keys);
% Plot points and covariance matrices
index = 1;
for i = 0:keys.size-1
    key = keys.at(i);
    p = values.at(key);
    if isa(p, 'gtsam.Point3')
            points.cov{index} = marginals.marginalCovariance(key);
            points.p{index} = [
                p.x
                p.y
                p.z];
            index = index + 1;
    end
end

end

