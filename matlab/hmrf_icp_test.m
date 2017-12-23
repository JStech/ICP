function [results] = hmrf_icp_test(first_axis_i, last_axis_i, angles)
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

% just make sure these are defined before loading the point clouds
assert(1 <= first_axis_i && first_axis_i <= last_axis_i && ...
    last_axis_i <= size(axes, 1));
%load_cloud;
fprintf('Loading clouds and poses\n')
load_data;
load_poses;
%load_other_cloud;
%load_other_poses;

params = icp_params;
params.h = 240;
params.w = 320;
params.debug = 1;
params.icp_iter_max = 50;
params.em_iter_max_start = 600;
params.em_iter_max = 20;
params.verbose = false;

ref_frame = 70;
src_frames = [27 53 247 257 260 267 273 287 334 370 394 418 422 434 455 464 ...
    517 568 628 647 653 662 666 677 686 689 693 703 706 719 726 747 803 844 ...
    878 879 889 897 903 936] + ref_frame;
%src_frames = 763;
%ref_frame = 156;
%src_frames = [15 35 38 39 41 47 89 129 133 135 148 150 151 154 155 157 158 ...
%159 160 162 163 167 171 181 184 190 197 204 205 219 256 271 282 290 301 313 ...
%344 355 363 365 382 417 465 471 481 489 513 518 554 573];

c1 = downsample(unproject(getcloud(ref_frame)), 640, 480, 2);
origin = mean(c1, 'omitnan');
origin(4) = 0;
c1 = c1 - origin;

results.params = {};
results.all = {};
results.pct = {};
results.sigma = {};
results.x84 = {};
results.dynamic = {};
results.hmrf = {};

for angle_i=1:length(angles)
  angle = angles(angle_i);
  for axis_i=[first_axis_i:last_axis_i]
    axis = axes(axis_i,:)';
    t_init = eye(4);
    t_init(1:3,1:3) = aa2mat(axis, angle);
    params.t_init = t_init;
    for src_frame_i=1:length(src_frames)
      src_frame = src_frames(src_frame_i);
      c2 = downsample(unproject(getcloud(src_frame)), 640, 480, 2);
      true_tf = inv(pmats{ref_frame})*pmats{src_frame};
      c2_t = (true_tf*c2')' - origin;
      ol = calculate_overlap(c1, c2_t);

      fprintf('%2d %8.5f %4d %8.5f\n', axis_i, angle, src_frame, ol);

      results.params{angle_i, axis_i, src_frame_i} = [src_frame, ol, angle];
      for mode = {'all' 'pct' 'sigma' 'x84' 'dynamic'}
        m = mode{1};
        params.mode = m;
        tic;
        [tf iters] = icp(c1, c2_t, params);
        elapsed = toc;
        tf_log = se3log(tf);
        results.(m){angle_i, axis_i, src_frame_i} = [iters, elapsed, norm(tf_log(1:3)), norm(tf_log(4:6))];
      end
      try
        tic;
        [tf data] = hmrf_icp(c1, c2_t, params);
        elapsed = toc;
        tf_log = se3log(tf);
        results.hmrf{angle_i, axis_i, src_frame_i} = [data.icp_iters, ...
          elapsed, norm(tf_log(1:3)), norm(tf_log(4:6)), data.em_iters];
      catch ae
        results.hmrf{angle_i, axis_i, src_frame_i} = nan;
      end
    end
  end
end
end
