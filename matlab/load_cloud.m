if ~exist('cloud') || size(cloud.Location, 1) ~= 1088
  cloud = pcread('../data/shark_bot_01.all_frames.pcd');
end

project = @(pts) pts(:,1:3)./pts(:,4);
unproject = @(pts) [pts ones(size(pts, 1), 1)];
getcloud = @(i) squeeze(cloud.Location(i,:,:));
