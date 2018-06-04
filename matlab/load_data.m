if strcmp('kitti', dataset)
  getcloud = @(i) getkittidata(i, 0);
else
  switch dataset
  case 'desk'
    n = 595;
    file = '../data/freiburg1_desk.mat';
  case 'room'
    n = 1360;
    file = '../data/freiburg1_room.mat';
  case 'shark'
    n = 1088;
    file = '../data/shark_bot_01.all_frames.mat';
  end

  if ~exist('cloudLoc') || size(cloudLoc, 1) ~= n
    load(file);
    assignin('base', 'cloudLoc', cloudLoc);
  end

  getcloud = @(i) squeeze(cloudLoc(i,:,:));

  clear file
end

unproject = @(pts) [pts ones(size(pts, 1), 1)];
project = @(pts) pts(:,1:3)./pts(:,4);
