load_cloud

c1 = unproject(getcloud(70));
tf_i = zeros(4, 4, 100);
tf_m = zeros(4, 4, 100);
for i=4:4:400
  c2 = unproject(getcloud(70+i));
  tf_i(:,:,i/4) = icp(c1, c2, 0.5, eye(4), 200, false);
  tf_m(:,:,i/4) = mycp(c1, c2, eye(4), 200, false);
  disp(sprintf('%d %f', i, sum(sum(abs(eye(4) - tf_i(:,:,i/4)*inv(tf_m(:,:,i/4)))))));
end
