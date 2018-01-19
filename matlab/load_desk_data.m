if ~exist('cloudLoc') || size(cloudLoc, 1) ~= 595
  load('../data/freiburg1_desk.mat');
  assignin('base', 'cloudLoc', cloudLoc);
end

project = @(pts) pts(:,1:3)./pts(:,4);
unproject = @(pts) [pts ones(size(pts, 1), 1)];
getcloud = @(i) squeeze(cloudLoc(i,:,:));
