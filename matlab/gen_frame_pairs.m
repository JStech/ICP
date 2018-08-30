cloudLoc = points;
unproject = @(pts) [pts ones(size(pts, 1), 1)];
pmats = arrayfun(@(i) pos2mat(poses(i,:)), [1:size(poses, 1)], 'UniformOutput', false);
getcloud = @(i) squeeze(cloudLoc(i,:,:));
n = size(cloudLoc, 1);
step = 1;
overlaps = nan(n);
for i=1:step:n
  ref_frame = i;
  c1 = unproject(getcloud(ref_frame));
  overlaps(i, i) = 1;
  tic;
  for j=i+step:step:n
    src_frame = j;
    c2 = unproject(getcloud(src_frame));
    true_tf = inv(pmats{ref_frame})*pmats{src_frame};
    c2_t = (true_tf*c2')';
    ol = calculate_overlap(c1, c2_t);
    overlaps(i, j) = ol;
    overlaps(j, i) = ol;
    %fprintf('%d %d %f\n', ref_frame, src_frame, ol);
  end
  toc
end
