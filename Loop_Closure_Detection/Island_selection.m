%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Roger (Aohan) Mei
% Date : 04/20/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function matched_island = Island_selection(v_t, temp_group, v_pre)
    H = zeros(length(temp_group),1);
    % We first need to calculate out the H value of each island
    for i  = 1:length(temp_group)
        temp_island = temp_group{i};
        sum = 0
        for j  = 1:length(temp_island)
            v_t
            temp_island{j}.Descriptor
            v_pre
            sum  = sum + normalized_similarity(v_t, temp_island{j}.Descriptor, v_pre);
        end
        H(i) = sum;
    end
    [max_H, index] = max(H);
    matched_island = temp_group(index);
end