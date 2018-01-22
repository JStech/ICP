raw_poses = dlmread('../data/freiburg1_room.poses', ' ', 3, 0);
ts_file = fopen('../data/freiburg1_room_depth_timestamps.csv');
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

q = poses(:,1:4);
t = poses(:,5:7);

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
