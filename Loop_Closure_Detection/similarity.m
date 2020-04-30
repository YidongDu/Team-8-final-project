%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Roger (Aohan) Mei
% Date : 04/20/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Define the similarity function
function score = similarity(v1,v2)
    score = 1 - 1/2*norm(abs(v1/norm(v1)-v2/norm(v2)));
end