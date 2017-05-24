icp;

v = [0; 0; -1];
th = pi/6;
m = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1];
assert(all(all(abs(aa2mat(v, th) - m)<1e-8)));
[v_test th_test] = mat2aa(m);
assert(all(abs(v_test - v)<1e-8));
assert(all(abs(th_test - th)<1e-8));

v = [ 0.0854358; 0.8543577; 0.5126146 ];
th = 2.8031185;
m = [ -0.929078, -0.0283688, 0.3687943;
0.3120568, 0.4751773, 0.822695;
-0.1985816, 0.8794326, -0.4326241 ];
[v_test th_test] = mat2aa(m);
m_test = aa2mat(v, th);
assert(all(abs(m_test(:) - m(:))<1e-5));
assert(all(abs(v_test - v)<1e-5));
assert(all(abs(th_test - th)<1e-5));

ref = [8.1472, 0.9754, 1.5761, 1;
       9.0579, 2.7850, 9.7059, 1;
       1.2699, 5.4688, 9.5717, 1;
       9.1338, 9.5751, 4.8538, 1;
       6.3236, 9.6489, 8.0028, 1];

src = [-10.7360,   1.1373,  -6.2434, 1;
       -23.9008,   2.6802,   4.0082, 1;
       -13.6096,  -0.6760,  16.4309, 1;
       -19.2731, -13.2004,   1.7711, 1;
       -19.7536, -10.6983,   9.8199, 1];

m = localize(ref, src, true);

assert(all(all(abs(ref' - m*src') < 1e-3)));

if ~exist('cloud')
  cloud = pcread('../data/shark_bot_01.few_frames.pcd');
end
c1 = squeeze(cloud.Location(:,1,:));
c1 = [c1 ones(size(c1, 1), 1)];
c2 = squeeze(cloud.Location(:,2,:));
c2 = [c2 ones(size(c1, 1), 1)];

if ~exist('T')
  T = icp(c1, c2, 20, eye(4), 50, false);
end

c2_T = (T*c2')';
th = 0.2;
v = [0 0 1];
R = [cos(th) + v(1)^2*(1-cos(th)), v(1)*v(2)*(1-cos(th))-v(3)*sin(th), v(1)*v(3)*(1-cos(th)) + v(2)*sin(th);
v(2)*v(1)*(1-cos(th))+v(3)*sin(th), cos(th)+v(2)^2*(1-cos(th)), v(2)*v(3)*(1-cos(th))-v(1)*sin(th);
v(3)*v(1)*(1-cos(th))-v(2)*sin(th), v(3)*v(2)*(1-cos(th)) + v(1)*sin(th), cos(th) + v(3)^2*(1-cos(th))];

R = [R zeros(3, 1);
  zeros(1, 3) 1];
c2_rot = (R*c2_T')';

if ~exist('T2')
  T2 = icp(c1, c2_rot, 20, eye(4), 500, false);
end
