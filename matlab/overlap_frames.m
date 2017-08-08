load_cloud
load_poses

start_frame = 70;
frame_step = 3;
num_steps = 200;
overlaps = zeros(1, num_steps);

c1 = unproject(getcloud(start_frame));

for i=1:num_steps;
  c2 = unproject(getcloud(start_frame + i*frame_step));
  c2_t = ((inv(pmats{start_frame})*pmats{start_frame + i*frame_step})*c2')';
  overlaps(i) = calculate_overlap(c1, c2_t);
end
