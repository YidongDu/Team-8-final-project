%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Roger (Aohan) Mei
% Date : 04/20/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% This function will check the geometric consistency of the two loop closure Candidate images
function result = Geometric_consistency_check(I1,I2)
    points1 = detectHarrisFeatures(I1);
    points2 = detectHarrisFeatures(I2);
    [f1, vpts1] = extractFeatures(I1, points1);
    [f2, vpts2] = extractFeatures(I2, points2);
    indexPairs = matchFeatures(f1, f2) ;
    if size(indexPairs,1) >= 12
        result = 1;
    else
        result = 0;
    end
end