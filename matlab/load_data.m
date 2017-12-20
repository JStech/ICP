if ~exist('cloudLoc') || size(cloudLoc, 1) ~= 1088
  load('../data/shark_bot_01.all_frames.mat');
  assignin('base', 'cloudLoc', cloudLoc);
end

project = @(pts) pts(:,1:3)./pts(:,4);
unproject = @(pts) [pts ones(size(pts, 1), 1)];
getcloud = @(i) squeeze(cloudLoc(i,:,:));
