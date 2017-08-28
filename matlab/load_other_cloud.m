if ~exist('cloud') || size(cloud.Location, 2) ~= 42
  cloud = pcread('../data/freiburg1_desk.pcd');
end

project = @(pts) pts(:,1:3)./pts(:,4);
unproject = @(pts) [pts ones(size(pts, 1), 1)];
getcloud = @(i) squeeze(cloud.Location(:,i,:));
