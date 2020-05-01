function [Maha_mat,L,landmark_class] = da_nn(Measurements, points, pose, sensor_noise, landmark_class)
Maha_mat = [];
L = [];
chi_score = zeros(size(Measurements.p,2),2);

for k = 1:size(Measurements.p,2)
    for i = 1:length(points)
        Mahalanobis = Maha(Measurements.range(1,k),points{i}.p,points{i}.cov,...
                      pose.p,pose.cov(1:3,1:3),sensor_noise);
        Maha_mat(i,k)=Mahalanobis;
    end
end

for j = 1:size(Measurements.p,2)
    if length(Maha_mat) == 0
        L(j,1) = 0;
    else
        [M,I] = min(Maha_mat(:,j));
        chi_score(j,:) = [M,I];
        if M < chi2inv(0.9,1) %the 90th percentile for the chi-square distribution with 1 degree of freedom
            L(j,1) = points{I}.index;
            if Measurements.score(j) > landmark_class(2,points{I}.index)
                landmark_class(1, points{I}.index) = Measurements.class(j);
                landmark_class(2, points{I}.index) = Measurements.score(j);
            end
        else
            L(j,1) = 0;
        end
    
    end
       
end

for i = 1:size(Maha_mat,1)
    if sum(chi_score(:,2)==i) > 1
        chi_min = min(chi_score(chi_score(:,2)==i,1));
        for j = 1:size(Measurements.p,2)
            if (chi_score(j,2)==i) && (chi_score(j,1)>chi_min)
                L(j,1) = -1;
            end
        end
    end
end