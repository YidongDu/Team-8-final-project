%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Roger (Aohan) Mei
% Date : 04/20/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% This function construct the matching island
% Dictionary data structure specification:
% The element in the dictionary should have such properties:
% Descriptor, Slot_number, etc
function group = match_grouping(v_t, dict, v_pre)
    % We first need to sort out all of the loop-closure candidates from the
    % dictionary. alpha is the threshold which need to be tuned.
    alpha = 0.5;
    candidates = {};
    for i = 1 : length(dict)
        % Retrieve the descriptor
        v_i = dict{i}.Descriptor;
        % Calculate the normalized similarity score
        score = normalized_similarity(v_t, v_i, v_pre);
        % Justify loop-closure candidate by comparing the normalized
        % similarity score with the threshold alpha.
        score
        if score >= alpha
            candidates{length(candidates)+1} = dict{i};
        end
    end
    if length(candidates) >= 2
        % We then need to seperate the candidates into different islands
        temp_group = Island_construction(candidates);
        % At last we need to select out the island with the largest H value.
        matched_island = Island_selection(v_t, temp_group);
        group = matched_island;
    else
        group = [];
    end
end