classdef icp_params
  properties
    do_scale = false;
    dt_thresh = 0.01;
    dth_thresh = 0.001;
    scale_thresh = 0.001;
    t_init = eye(4);
    icp_iter_max = 100;
    em_iter_max_start = 1000;
    em_iter_max = 100;
    % Outlier distribution: normal or logistic
    em_outlier_dist = 'logistic';
    % Neighbor structure: grid or unstructured
    neighbor_structure = 'grid';
    neighbor_sigma = 1e-1;
    max_neighbors = 10;
    % ICP mode: all, pct, sigma, dynamic, x84
    mode = 'all';
    D = 1.;
    beta = 2.;
    gamma = 0.;
    debug = false;
    verbose = false;
    make_animation = false;
    w = 640;
    h = 480;
    pyramid_levels = 3;
  end
end
