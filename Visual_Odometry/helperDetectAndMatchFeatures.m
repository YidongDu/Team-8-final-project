function [currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(...
    prevFeatures, I)

numPoints = 150;

% Detect and extract features from the current image.
currPoints   = detectSURFFeatures(I, 'MetricThreshold', 500);
currPoints   = selectUniform(currPoints, numPoints, size(I));
currFeatures = extractFeatures(I, currPoints, 'Upright', true);

% Match features between the previous and current image.
indexPairs = matchFeatures(prevFeatures, currFeatures, 'Unique', true);
