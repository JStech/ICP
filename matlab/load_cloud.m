cloud = pcread('../data/shark_bot_01.few_frames.pcd');
if ~exist('f1')
  f1 = 1;
end
if ~exist('f2')
  f2 = 2;
end
c1 = squeeze(cloud.Location(:,f1,:));
c1 = [c1 ones(307200, 1)];
c2 = squeeze(cloud.Location(:,f2,:));
c2 = [c2 ones(307200, 1)];
