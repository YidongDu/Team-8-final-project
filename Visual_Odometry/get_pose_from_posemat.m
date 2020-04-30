function groundtruth_poses = get_pose_from_posemat(pose_matrices)
    groundtruth_poses.position = {};
    groundtruth_poses.orientation = {};
    Coordinate_transfer = [0,-1,0;
                           0,0,-1;
                           1,0,0];
    for pose_index = 1:length(pose_matrices)
        groundtruth_poses.Location{pose_index} = (Coordinate_transfer*pose_matrices{pose_index}(1:3,4))';
%         groundtruth_poses.Location{pose_index} = (pose_matrices{pose_index}(1:3,4))';
        groundtruth_poses.Orientation{pose_index} = pose_matrices{pose_index}(1:3,1:3);
    end
end