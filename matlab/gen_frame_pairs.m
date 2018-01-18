load_other_cloud;
load_other_poses;
cloudLoc = cloud.Location;

n = size(cloudLoc, 2);
step = 6;
overlaps = nan(n);
for i=1:step:n
  ref_frame = i
  c1 = downsample(unproject(getcloud(ref_frame)), 640, 480, 2);
  origin = mean(c1, 'omitnan');
  origin(4) = 0;
  c1 = c1 - origin;
  overlaps(i, i) = 1;
  for j=i+step:step:n
    src_frame = j;
    c2 = downsample(unproject(getcloud(src_frame)), 640, 480, 2);
    true_tf = inv(pmats{ref_frame})*pmats{src_frame};
    c2_t = (true_tf*c2')' - origin;
    ol = calculate_overlap(c1, c2_t);
    overlaps(i, j) = ol;
    overlaps(j, i) = ol;
  end
end
