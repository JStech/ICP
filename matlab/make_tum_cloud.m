for dataset = {'freiburg1_desk', 'freiburg1_desk2', 'freiburg1_floor', ...
  'freiburg1_room', 'freiburg2_360_hemisphere', 'freiburg2_360_kidnap', ...
  'freiburg2_desk', 'freiburg2_large_no_loop', 'freiburg2_large_with_loop', ...
  'freiburg3_long_office_household'}
  fx = 525.0; % focal length x
  fy = 525.0; % focal length y
  cx = 319.5; % optical center x
  cy = 239.5; % optical center y
  h = 480;
  w = 640;

  factor = 5000;

  files = dir(['../data/all_tum/rgbd_dataset_' dataset{1} '/depth/*.png']);
  n = min(length(files), 1000);

  skip = 10;
  points = zeros(floor(n/skip), h/2*w/2, 3);
  for i = skip:skip:n
    depth_image = imread([files(i).folder '/' files(i).name]);
    for v = 2:2:h
      for u = 2:2:w
        if depth_image(v, u)==0
          points(i/skip, (v/2-1)*w/2+u/2, :) = [nan nan nan];
        else
          Z = double(depth_image(v, u)) / factor;
          X = (u - cx) * Z / fx;
          Y = (v - cy) * Z / fy;
          assert((X~=0) || (Y~=0) || (Z~=0))
          points(i/skip, (v/2-1)*w/2+u/2, :) = [X Y Z];
        end
      end
    end
  end

  save(['../data/tum_' dataset{1} '.mat'], 'points', '-v7.3')
  clear
end
%cloud = pointCloud(points);
%pcwrite(cloud, '/Users/john/arpg/ICP/data/freiburg1_desk.pcd', 'Encoding', 'binary');
