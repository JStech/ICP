for dataset = {'freiburg1_desk', 'freiburg1_desk2', 'freiburg1_floor', ...
  'freiburg1_room', 'freiburg2_360_hemisphere', 'freiburg2_360_kidnap', ...
  'freiburg2_desk', 'freiburg2_large_no_loop', 'freiburg2_large_with_loop', ...
  'freiburg3_long_office_household'}
  f = fopen(['../data/all_tum/rgbd_dataset_' dataset{1} '/depth.txt']);
  timestamps = textscan(f, '%f %s', 'CommentStyle', '#');
  fclose(f);
  timestamps = timestamps{1};

  raw_poses = dlmread(['../data/all_tum/rgbd_dataset_' dataset{1} ...
  '/groundtruth.txt'], ' ', 3, 0);
  n_poses = min(length(timestamps), 1000);

  skip = 10;
  for i=skip:skip:n_poses
    next_pose_i = find(raw_poses(:,1) > timestamps(i), 1);
    delta_t = raw_poses(next_pose_i, 1) - raw_poses(next_pose_i-1, 1);
    c = abs(raw_poses(next_pose_i-1:next_pose_i, 1) - timestamps(i))/delta_t;
    poses(i/skip, :) = c(2)*raw_poses(next_pose_i-1, 2:end) + c(1)*raw_poses(next_pose_i, 2:end);
  end
  poses = [poses(:,4:7) poses(:,1:3)];
  save(['../data/tum_' dataset{1} '_poses.mat'], 'poses', '-v7.3')
  clear
end
