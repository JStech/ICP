f = fopen('../data/rgbd_dataset_freiburg1_room/depth.txt');
timestamps = textscan(f, '%f %s', 'CommentStyle', '#');
fclose(f);
timestamps = timestamps{1};

raw_poses = dlmread('../data/rgbd_dataset_freiburg1_room/groundtruth.txt', ' ', 3, 0);
n_poses = length(timestamps)

for i=1:n_poses
  next_pose_i = find(raw_poses(:,1) > timestamps(i), 1);
  delta_t = raw_poses(next_pose_i, 1) - raw_poses(next_pose_i-1, 1);
  c = abs(raw_poses(next_pose_i-1:next_pose_i, 1) - timestamps(1))/delta_t;
  poses(i, :) = c(2)*raw_poses(next_pose_i-1, 2:end) + c(1)*raw_poses(next_pose_i, 2:end);
end
