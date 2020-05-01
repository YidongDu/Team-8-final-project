function [gt, Measurements, Transform] = Data_Parsing(Landmark_location)
load('relative_pose.mat');
load('true_pose.mat');

for j = 1:2762
    Measurements{j}.centroid_x = [];
    Measurements{j}.centroid_y =[];
    Measurements{j}.p =[];
    Measurements{j}.range =[];
    Measurements{j}.class = [];
    Measurements{j}.score = [];
    if j < 2762
        Transform{j}.R = Relative{j}(1:3,1:3);
        Transform{j}.p = Relative{j}(1:3,4);
    end
    gt{j}.R = pose_matrices{j}(1:3,1:3);
    gt{j}.p = pose_matrices{j}(1:3,4);
    
    temp = strcat('000000000',num2str(j-1));
    temp = temp(end-9:end);
    file = strcat(Landmark_location,'\',temp,'.csv');
    try
        M = csvread(file);   
    catch
        M = [];
    end

    [r,~] = size(M);
    if r == 0 
        Measurements{j} = [];
    else
        for i = 1:1:r
            %range = sqrt((M(i,16)-5)^2 + M(i,17)^2 + M(i,18)^2);
            range = sqrt(M(i,16)^2 + M(i,17)^2 + M(i,18)^2);
            Measurements{j}.centroid_x = [Measurements{j}.centroid_x M(i,1)];
            Measurements{j}.centroid_y = [Measurements{j}.centroid_y M(i,2)];
            %Measurements{j}.p = [Measurements{j}.p [(M(i,18)-5); M(i,16);M(i,17)]];
            Measurements{j}.p = [Measurements{j}.p [M(i,18); M(i,16);M(i,17)]];
            Measurements{j}.range = [Measurements{j}.range range];
            Measurements{j}.class = [Measurements{j}.class M(i,6)];
            Measurements{j}.score = [Measurements{j}.score M(i,5)];
        end
    end
end

