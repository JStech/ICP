function [results] = hmrf_icp_tum_test(pairs, modes, angles)

switch nargin
case 0
  error('pairs is required')
case 1
  modes = {'all' 'pct' 'sigma' 'x84' 'dynamic' 'hmrf' 'goicp' 's4pcs'};
  angles = [pi/30 pi/15 pi/10];
case 2
  angles = [pi/30 pi/15 pi/10];
case 3
otherwise
  error('Too many arguments');
end

global cloudLoc;
axes = [0.7295   -0.4166    0.5425
        0.8532   -0.5095    0.1116
        0.6788   -0.2187   -0.7010
       -0.2478    0.7270    0.6404
       -0.7575    0.2070    0.6191
        0.9449    0.3079   -0.1111
       -0.5756   -0.3254    0.7502
        0.0405    0.9310    0.3629
        0.8946   -0.3116    0.3203
        0.5176    0.4179    0.7466
        0.8120   -0.4022   -0.4230
        0.0548   -0.8942    0.4442
       -0.6707   -0.6396    0.3755
       -0.8057    0.5299   -0.2645
       -0.9671   -0.1616    0.1965
        0.8934   -0.0745   -0.4431];

params = icp_params;
params.h = 240;
params.w = 320;
params.debug = 1;
params.icp_iter_max = 50;
params.pyramid_levels = 4;
params.em_iter_max_start = 150;
params.em_iter_max = 5;
params.verbose = false;

results.params = {};
results.all = {};
results.pct = {};
results.sigma = {};
results.x84 = {};
results.dynamic = {};
results.hmrf = {};
results.goicp = {};

unproject = @(pts) [pts ones(size(pts, 1), 1)];
project = @(pts) pts(:,1:3)./pts(:,4);

last_dataset = '';
for i_p = 1:size(pairs, 1)
  dataset = pairs{i_p, 1};
  if (~strcmp(dataset, last_dataset))
    load(['../data/tum_' dataset '.mat']);
    load(['../data/tum_' dataset '_poses.mat']);
    pmats = arrayfun(@(i) pos2mat(poses(i,:)), [1:size(poses, 1)], 'UniformOutput', false);
    last_dataset = dataset;
  end

  for angle_i=1:length(angles)
    angle = angles(angle_i);
    axis_i = randi(length(axes));
    axis = axes(axis_i, :)';
    t_init = eye(4);
    t_init(1:3,1:3) = aa2mat(axis, angle);
    params.t_init = t_init;

    ol = pairs{i_p, 2};
    ref_frame = pairs{i_p, 3};
    src_frame = pairs{i_p, 4};

    c1 = unproject(squeeze(points(ref_frame,:,:)));
    origin = [mean(c1(:,1:3), 'omitnan'), 0];
    c1_z = c1 - origin;
    c2 = unproject(squeeze(points(src_frame,:,:)));
    true_tf = inv(pmats{ref_frame})*pmats{src_frame};
    c2_t = (true_tf*c2')' - origin;

    fprintf('%4d %4d %8.5f %8.5f %3d %s\n', ref_frame, src_frame, ol, angle, axis_i, dataset);

    results.params{angle_i, i_p} = ...
        [ref_frame, src_frame, ol, angle, axis_i];
    for mode = modes
      m = mode{1};
      params.mode = m;
      if strcmp(m, 'goicp')
        if angle_i==1
          [tf elapsed] = goicp(c1_z, c2_t, params);
          iters = 0;
        end
      elseif strcmp(m, 's4pcs')
        if angle_i==1
          [tf elapsed] = s4pcs(c1_z, c2_t, ol, params);
          iters = 0;
        end
      elseif strcmp(m, 'hmrf')
        try
          tic;
          [tf data] = hmrf_icp(c1_z, c2_t, params);
          elapsed = toc;
          tf_log = se3log(tf);
          results.hmrf{angle_i, i_p} = [data.icp_iters, ...
            elapsed, norm(tf_log(1:3)), norm(tf_log(4:6)), data.em_iters];
        catch ae
          results.hmrf{angle_i, i_p} = nan;
        end
      else
        tic;
        [tf iters] = icp(c1_z, c2_t, params);
        elapsed = toc;
      end
      if ~strcmp(m, 'hmrf')
        tf_log = se3log(tf);
        results.(m){angle_i, i_p} = [iters, elapsed, norm(tf_log(1:3)), norm(tf_log(4:6))];
      end
    end
  end
end
end
