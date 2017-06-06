load_cloud
load_poses

params = icp_params;
params.iter_max = 200;

c1 = unproject(getcloud(70));
tf.all = zeros(4, 4, 100);
tf.pct = zeros(4, 4, 100);
tf.sigma = zeros(4, 4, 100);
tf.dynamic = zeros(4, 4, 100);
tf.my = zeros(4, 4, 100);
iters.all = zeros(1, 100);
iters.pct = zeros(1, 100);
iters.sigma = zeros(1, 100);
iters.dynamic = zeros(1, 100);
iters.my = zeros(1, 100);

for i=4:4:400
  c2 = unproject(getcloud(70+i));
  for mode = {'all' 'pct' 'sigma' 'dynamic'}
    m = mode{1};
    params.mode = m;
    [tf.(m)(:,:,i/4) iters.(m)(i/4)] = icp(c1, c2, params);
  end
  [tf.my(:,:,i/4) iters.my(i/4)] = mycp(c1, c2, params);
  true_tf = inv(pmats{70})*pmats{70+i};
  err_my = se3log(inv(tf.my(:,:,i/4))*true_tf);
  err_all = se3log(inv(tf.all(:,:,i/4))*true_tf);
  err_pct = se3log(inv(tf.pct(:,:,i/4))*true_tf);
  err_sigma = se3log(inv(tf.sigma(:,:,i/4))*true_tf);
  err_dynamic = se3log(inv(tf.dynamic(:,:,i/4))*true_tf);
  disp(sprintf('%3d %8.5f, %8.5f; %8.5f, %8.5f; %8.5f, %8.5f; %8.5f, %8.5f; %8.5f, %8.5f;', i, ...
    norm(err_my(1:3)), norm(err_my(4:6)), ...
    norm(err_all(1:3)), norm(err_all(4:6)), ...
    norm(err_pct(1:3)), norm(err_pct(4:6)), ...
    norm(err_sigma(1:3)), norm(err_sigma(4:6)), ...
    norm(err_dynamic(1:3)), norm(err_dynamic(4:6))));
end
