function [weight] = SS_Weight(Maha_mat, L, candidate_points, landmark_class, Measurements)
%load confusion matrix, the ith-jth of which shows the probility of
%detecting class j as class i
load('confusion_mtx.mat');

%compute weight
[r,c] = size(Maha_mat);
weight = zeros(r,c);
mea_list = [];
mark_list = [];
for i = 1:c
    if (L(i) ~= 0) && (L(i) ~= -1)
        mea_list = [mea_list i];
        for j = 1:r
            if L(i) == candidate_points{j}.index
                mark_list = [mark_list j];
            end
        end
    end
end

if ~isempty(mea_list)
    kernel = perms(mark_list);
    kernel(:,end+1) = ones(size(kernel, 1),1);
    for i = 1:size(kernel, 1)
        for j = 1:length(mea_list)
            mark = kernel(i, j);
            mark_class = landmark_class(1, candidate_points{mark}.index);
            mea = mea_list(j);
            mea_class = Measurements.class(mea);
            kernel(i,end) = kernel(i,end) * conf_mtx(mea_class+1,mark_class+1) * Measurements.score(mea) * exp(-0.5*Maha_mat(mark,mea));
        end
    end
    kernel(:,end) = kernel(:,end)./sum(kernel(:,end));
    for i = 1:length(mea_list)
        mea = mea_list(i);
        for j = 1:r
            if ismember(j,mark_list)
                weight(j,mea) = sum(kernel(kernel(:,i)==j, end));
            end
        end
    end
end
