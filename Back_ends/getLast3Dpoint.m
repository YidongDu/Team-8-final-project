function point = getLast3Dpoint(values, marginals)
%getLast3Dpoint get the 3Dpoints(landmark) location and covariance from the result
%   point.cord     saves the coordinate of the last point
%   point.cov      saves the covariance of the last point
import gtsam.*
keys = KeyVector(values.keys);
% Plot points and covariance matrices

for i = keys.size-1
    key = keys.at(i);
    p = values.at(key);
    if isa(p, 'gtsam.Point3')
            point.cov = marginals.marginalCovariance(key);
            point.p = [
                p.x
                p.y
                p.z];
    end
end

end

