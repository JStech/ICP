if ~exist('cloud') || size(cloud.Location, 2) ~= 595
  load('../data/freiburg1_desk.mat');
  assignin('base', 'cloudLoc', cloudLoc);
end

project = @(pts) pts(:,1:3)./pts(:,4);
unproject = @(pts) [pts ones(size(pts, 1), 1)];
getcloud = @(i) squeeze(cloud.Location(i,:,:));
