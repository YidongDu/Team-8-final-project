%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Roger (Aohan) Mei
% Date : 04/20/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;clc;
% Read the image file
images = imageDatastore('/Users/rogermei/Desktop/Umich 20 Winter/EECS_568/Final_project/visual_odometry/image_02/')
% Construct the dictionary
temp_tree = load('word_tree.mat');
tree = temp_tree.tree;
% Create a dictionary
dict = {};
% Loop closure detection
for idx = 1 : numel(images.Files)
    idx
    I = readimage(images, idx);
    I = rgb2gray(I);
    % Extract its BRIEF Descriptor
    BRIEF_descriptor = BRIEF_descriptor_construction(I);
    BRIEF_descriptor = BRIEF_descriptor';
    % Traverse the tree to get the vector v_t
    v_t = calculate_vt(BRIEF_descriptor, tree);
    % Construct the dictionary
    temp.Slot_number = idx;
    temp.Descriptor = v_t;
    dict{length(dict) + 1} = temp;
    % Calculate v_pre
    if idx >= 3
        % Find the matched candidates
        candidates = match_grouping(v_t,dict,v_pre);
        if isempty(candidates)
        else
            % Construct the island
            temp_group = Island_construction(candidates);
            % Select the island with highes H score
            matched_island = Island_selection(v_t, temp_group, v_pre);
            % Implement Geometric consistency check!
            loop_closure_candidates = [];
            for i  = 1:length(matched_island)
                candidate_temp = matched_island(i);
                result = Geometric_consistency_check(I1,I2);
                if result == 1
                    loop_closure_candidates(length(loop_closure_candidates)+1) = candidate_temp;
                end
            end
        end
    end
    % Calculate v_pre
    v_pre = v_t;
end
