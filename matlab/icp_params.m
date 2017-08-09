classdef icp_params
  properties
    do_scale = false;
    dt_thresh = 0.01;
    dth_thresh = 0.001;
    scale_thresh = 0.001;
    t_init = eye(4);
    iter_max = 100;
    % ICP mode: all, pct, sigma, dynamic
    mode = 'all';
    D = 1.;
    beta = 2.;
    gamma = 0.;
    debug = false;
    verbose = false;
    w = 640;
    h = 480;
  end
end
