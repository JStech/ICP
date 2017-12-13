load_cloud
load_poses
%load_other_cloud
%load_other_poses
icp;

params = icp_params;
params.h = 240;
params.w = 320;
params.debug = 1;
params.iter_max_start = 600;
params.iter_max_inner = 20;
params.iter_max = 50;
params.make_animation = true;
params.verbose = true;

ref_frame = 70;
src_frame = 763;

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

c1 = downsample(unproject(getcloud(ref_frame)), 640, 480, 2);
origin = mean(c1, 'omitnan');
origin(4) = 0;
c1 = c1 - origin;

angle=pi/10;
axis = axes(axis_i,:)';
t_init = eye(4);
t_init(1:3,1:3) = aa2mat(axis, angle);
params.t_init = t_init;

c2 = downsample(unproject(getcloud(src_frame)), 640, 480, 2);
true_tf = inv(pmats{ref_frame})*pmats{src_frame};
c2_t = (true_tf*c2')' - origin;
ol = calculate_overlap(c1, c2_t);

fprintf('%2d %4d %8.5f\n', axis_i, src_frame, ol)

[tf iters iters_start] = hmrf_icp(c1, c2_t, params);
