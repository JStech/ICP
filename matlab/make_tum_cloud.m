fx = 525.0; % focal length x
fy = 525.0; % focal length y
cx = 319.5; % optical center x
cy = 239.5; % optical center y
h = 480;
w = 640;

factor = 5000;

files = dir('/Users/john/tum_data/rgbd_dataset_freiburg1_desk/depth/*.png');
n = length(files);

points = zeros(h*w, n, 3);
for i = 1:n
  depth_image = imread([files(i).folder '/' files(i).name]);
  for v = 1:h
    for u = 1:w
      Z = double(depth_image(v, u)) / factor;
      X = (u - cx) * Z / fx;
      Y = (v - cy) * Z / fy;
      points((v-1)*w+u, i,:) = [X Y Z];
    end
  end
end

cloud = pointCloud(points);
pcwrite(cloud, '/Users/john/arpg/ICP/data/freiburg1_desk.pcd', 'Encoding',
'binary');
