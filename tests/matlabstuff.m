points = [8.1472    9.0579    1.2699    9.1338    6.3236;
          0.9754    2.7850    5.4688    9.5751    9.6489;
          1.5761    9.7059    9.5717    4.8538    8.0028;
          1         1         1         1         1     ];
t = [0.7014; 1.6983; 1.8680];
v = [0.5388; 0.6015; 0.5899];
v_dual = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
theta = pi/12;
R = [v*v' + cos(theta)*(eye(3) - v*v') + sin(theta)*v_dual, zeros(3, 1); zeros(1, 3) 1];
T = R * (eye(4) + [zeros(4, 3) [t; 0]]);

src_pts = T*points;

x1 = points(1,:);
y1 = points(2,:);
z1 = points(3,:);
x2 = src_pts(1,:);
y2 = src_pts(2,:);
z2 = src_pts(3,:);
