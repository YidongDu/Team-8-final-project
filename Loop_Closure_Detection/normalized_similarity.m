%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Roger (Aohan) Mei
% Date : 04/20/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% This function defines the normalized similarity score
function score = normalized_similarity(v_t,v_tj,v_pre)
    
    score = similarity(v_t,v_tj)/similarity(v_t, v_pre);
end