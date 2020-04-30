%FAST_Feature;
%feature_matrix = double(features.Features);
train_set = load('word_tree.mat');
descriptor_saver = train_set.descriptor_saver;
feature_matrix = double(descriptor_saver);
[idx, C] = kmeans(feature_matrix,1); %compute centroid
tree = grow_tree_kmean_indexed(feature_matrix,C,5,3,0,length(feature_matrix),0);
