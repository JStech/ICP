load_cloud
load_poses

params = icp_params;

% frames for testing; overlaps 0.3251 0.5000 0.6993 0.8935 0.9844
ref_frame = 70;
src_frames = 3*[89 158 71 35 4] + 70;
axes = [0.7295   -0.4166    0.5425
        0.8532   -0.5095    0.1116
        0.6788   -0.2187   -0.7010
       -0.2478    0.7270    0.6404
       -0.7575    0.2070    0.6191
        0.9449    0.3079   -0.1111];

c1 = unproject(getcloud(ref_frame));
origin = mean(c1, 'omitnan');
origin(4) = 0;
c1 = c1 - origin;

for src_frame=src_frames
  c2 = unproject(getcloud(src_frame));
  true_tf = inv(pmats{ref_frame})*pmats{src_frame};
  c2_t = (true_tf*c2')' - origin;

  for angle=[pi/30 pi/20 pi/10 pi/5]
    for axis=axes
      S = [0 -axis(3) axis(2); axis(3) 0 -axis(1); -axis(2) axis(1) 0];
      t_init = eye(4);
      t_init(1:3,1:3) = eye(3) + sin(angle)*S + (1-cos(angle))*S^2;
      params.t_init = t_init;
      for mode = {'all' 'pct' 'sigma' 'dynamic'}
        m = mode{1};
        params.mode = m;
        [tf iters] = icp(c1, c2_t, params);
        tf_log = se3log(tf);

        disp(sprintf('%3d %8.5f %10s %3d %8.5f %8.5f', src_frame, angle, m, iters, norm(tf_log(1:3)), norm(tf_log(4:6))))
      end
      [tf iters] = mycp(c1, c2_t, params);
      tf_log = se3log(tf);
      disp(sprintf('%8.5f %10s %3d %8.5f %8.5f', angle, 'hmrf', iters, norm(tf_log(1:3)), norm(tf_log(4:6))))
    end
  end
end
