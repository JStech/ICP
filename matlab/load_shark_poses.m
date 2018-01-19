poses = csvread('../data/shark_bot_01.poses');

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
