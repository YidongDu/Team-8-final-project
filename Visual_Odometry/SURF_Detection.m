clear;clc;
I = imread('Sample.png');
points = detectSURFFeatures(I);
[features, valid_points] = extractFeatures(I, points);
figure; imshow(I); hold on;
[B,idx] = maxk(valid_points.Metric,30);
valid_feature = zeros(length(idx), size(features,2));
for i = 1:length(idx)
    valid_feature(i,:) = features(idx(i),:);
end
plot(valid_points.selectStrongest(30),'showOrientation',true);
% plot(valid_points,'showOrientation',true);