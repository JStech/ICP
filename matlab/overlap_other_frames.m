load_other_cloud;
load_other_poses;
icp;

ref_frame = 156;
start_frame = 0;
frame_step = 1;
num_steps = 595-start_frame;
overlaps = zeros(1, num_steps);

c1 = unproject(getcloud(ref_frame));
c1 = downsample(c1, 640, 480, 2);

for i=1:num_steps;
  c2 = unproject(getcloud(start_frame + i*frame_step));
  c2 = downsample(c2, 640, 480, 2);
  c2_t = ((inv(pmats{ref_frame})*pmats{start_frame + i*frame_step})*c2')';
  overlaps(i) = calculate_overlap(c1, c2_t);
  fprintf(' %3d', i);
end
fprintf('\n')
