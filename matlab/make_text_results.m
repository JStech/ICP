load('../data/results.mat');
fprintf(['frame,overlap,angle,all_iters,all_time,all_t_err,all_r_err,'...
    'pct_iters,pct_time,pct_t_err,pct_r_err,sigma_iters,sigma_time,'...
    'sigma_t_err,sigma_r_err,x84_iters,x84_time,x84_t_err,x84_r_err,'...
    'dynamic_iters,dynamic_time,dynamic_t_err,dynamic_r_err,hmrf_iters,'...
    'hmrf_em_iters,hmrf_time,hmrf_t_err,hmrf_r_err\n']);
for angle_i=1:size(r.all, 1)
  for axis_i=1:size(r.all, 2)
    for src_frame_i=1:size(r.all, 3)
      frame = r.params{angle_i, axis_i, src_frame_i}(1);
      overlap = r.params{angle_i, axis_i, src_frame_i}(2);
      angle = r.params{angle_i, axis_i, src_frame_i}(3);
      all_iters = r.all{angle_i, axis_i, src_frame_i}(1);
      all_time = r.all{angle_i, axis_i, src_frame_i}(2);
      all_t_err = r.all{angle_i, axis_i, src_frame_i}(3);
      all_r_err = r.all{angle_i, axis_i, src_frame_i}(4);
      pct_iters = r.pct{angle_i, axis_i, src_frame_i}(1);
      pct_time = r.pct{angle_i, axis_i, src_frame_i}(2);
      pct_t_err = r.pct{angle_i, axis_i, src_frame_i}(3);
      pct_r_err = r.pct{angle_i, axis_i, src_frame_i}(4);
      sigma_iters = r.sigma{angle_i, axis_i, src_frame_i}(1);
      sigma_time = r.sigma{angle_i, axis_i, src_frame_i}(2);
      sigma_t_err = r.sigma{angle_i, axis_i, src_frame_i}(3);
      sigma_r_err = r.sigma{angle_i, axis_i, src_frame_i}(4);
      x84_iters = r.x84{angle_i, axis_i, src_frame_i}(1);
      x84_time = r.x84{angle_i, axis_i, src_frame_i}(2);
      x84_t_err = r.x84{angle_i, axis_i, src_frame_i}(3);
      x84_r_err = r.x84{angle_i, axis_i, src_frame_i}(4);
      dynamic_iters = r.dynamic{angle_i, axis_i, src_frame_i}(1);
      dynamic_time = r.dynamic{angle_i, axis_i, src_frame_i}(2);
      dynamic_t_err = r.dynamic{angle_i, axis_i, src_frame_i}(3);
      dynamic_r_err = r.dynamic{angle_i, axis_i, src_frame_i}(4);
      try
        hmrf_iters = r.hmrf{angle_i, axis_i, src_frame_i}(1);
        hmrf_time = r.hmrf{angle_i, axis_i, src_frame_i}(2);
        hmrf_t_err = r.hmrf{angle_i, axis_i, src_frame_i}(3);
        hmrf_r_err = r.hmrf{angle_i, axis_i, src_frame_i}(4);
        hmrf_em_iters = sum(r.hmrf{angle_i, axis_i, src_frame_i}(5:end));
      catch ae
        hmrf_iters = nan;
        hmrf_time = nan;
        hmrf_t_err = nan;
        hmrf_r_err = nan;
        hmrf_em_iters = nan;
      end

      fprintf(['%d,%8.5f,%8.5f,%d,%8.5f,%8.5f,%8.5f,%d,%8.5f,%8.5f,%8.5f,' ...
      '%d,%8.5f,%8.5f,%8.5f,%d,%8.5f,%8.5f,%8.5f,%d,%8.5f,%8.5f,%8.5f,%d,' ...
      '%d,%8.5f,%8.5f,%8.5f\n'], ...
      frame, overlap, angle, ...
      all_iters, all_time, all_t_err, all_r_err, ...
      pct_iters, pct_time, pct_t_err, pct_r_err, ...
      sigma_iters, sigma_time, sigma_t_err, sigma_r_err, ...
      x84_iters, x84_time, x84_t_err, x84_r_err, ...
      dynamic_iters, dynamic_time, dynamic_t_err, dynamic_r_err, ...
      hmrf_iters, hmrf_em_iters, hmrf_time, hmrf_t_err, hmrf_r_err);
    end
  end
end
