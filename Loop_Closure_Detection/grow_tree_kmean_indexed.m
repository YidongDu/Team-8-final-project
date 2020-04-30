%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Owen Winship
% Date : 04/27/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function tree = grow_tree_kmean_indexed(features,centroid,k,total_levels,level,initial_size,lf_index)
    
    new_level = level+1;
    tree.features = features;
    tree.centroid = centroid;
    size_f = size(features);
    num_features = size_f(1);
    tree.weight = log(initial_size/num_features);
    tree.word_vector = [];
    tree.lf_index = lf_index;
    
    
    if((num_features > k)&&(new_level <= total_levels))
        [idx, C] = kmeans(features,k);

        for i = 1:k
            childname = strcat('child',num2str(i));
            %node.(childname).features = node.features(idx == k);
            %node.(childname).centroid = C(k,:);
            new_features = features(idx == i,:);
            new_centroid = C(i,:);
            new_centroid = round(new_centroid);
            tree.(childname) = grow_tree_kmean_indexed(new_features,new_centroid,k,total_levels,new_level,initial_size,tree.lf_index);
            tree.lf_index = tree.(childname).lf_index;
        end
        for i = 1:k
            childname = strcat('child',num2str(i));
            tree.word_vector = [tree.word_vector;tree.(childname).word_vector];
        end
    else
        tree.word_vector = [tree.word_vector;[centroid,tree.weight]];
        tree.lf_index = tree.lf_index+1;
    end
end