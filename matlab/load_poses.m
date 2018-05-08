switch dataset
case 'desk'
  pose_file = '../data/freiburg1_desk.poses';
  ts_file = '../data/freiburg1_desk_depth_timestamps.csv';
case 'room'
  pose_file = '../data/freiburg1_room.poses';
  ts_file = '../data/freiburg1_room_depth_timestamps.csv';
case 'shark'
  pose_file = '../data/shark_bot_01.poses';
end

if strcmp(dataset, 'shark')
  poses = csvread(pose_file);
else
  raw_poses = dlmread(pose_file, ' ', 3, 0);
  ts_file = fopen(ts_file);
  timestamps = textscan(ts_file, '%f %s', 'CommentStyle', '#');
  fclose(ts_file);
  timestamps = timestamps{1};
  n_poses = size(timestamps, 1);
  poses = zeros(size(timestamps, 1), 7);
  next_pose_i = zeros(n_poses, 1);

  % interpolate
  for i=1:n_poses
    next_pose_i(i) = find(raw_poses(:,1) > timestamps(i), 1);
  end

  delta_t = raw_poses(next_pose_i, 1) - raw_poses(next_pose_i-1, 1);
  c = abs(raw_poses(next_pose_i, 1) - timestamps)./delta_t;
  poses = c.*raw_poses(next_pose_i-1, 2:end) + ...
          (1-c).*raw_poses(next_pose_i, 2:end);
  poses = [poses(:,4:7) poses(:,1:3)];
  poses(:,1:4) = poses(:, 1:4) ./ sqrt(sum(poses(:, 1:4).^2, 2));
end

pos2mat = @(p) [...
eye(4) + ...
2*[-p(2)^2-p(3)^2, p(1)*p(2), p(1)*p(3), 0;
p(1)*p(2), -p(1)^2-p(3)^2, p(2)*p(3), 0;
p(1)*p(3), p(2)*p(3), -p(1)^2-p(2)^2, 0;
0, 0, 0, 0] + ...
2*p(4) * [0, -p(3), p(2), 0;
p(3), 0, -p(1), 0;
-p(2), p(1), 0, 0;
0, 0, 0, 0]
] + ...
[zeros(3, 3) p(5:7)'; zeros(1, 4)];

pmats = arrayfun(@(i) pos2mat(poses(i,:)), [1:size(poses, 1)], 'UniformOutput', false);
clear raw_poses pose_file ts_file next_pose_i delta_t c timestamps n_poses i ans
