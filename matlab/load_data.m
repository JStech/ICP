if strcmp('kitti', dataset)
  if 0 == exist('kitti_sequence')
    kitti_sequence = 0;
  end
  getcloud = @(i) getkittidata(i, kitti_sequence);
  n_kitti_clouds = size(dir(sprintf('../data/dataset/sequences/%02d/velodyne/*.bin', kitti_sequence)), 1);
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
