function [T] = plot_icp_seq(clouds, tfs, ds)
  if 0==exist('ds')
    ds = 1;
  end
  figure;
  hold on
  T = eye(4);
  colors = {'red', 'green', 'blue'};

  pcshow(clouds{1}(1:ds:end,1:3), colors{mod(0, size(colors, 2))+1}, 'MarkerSize', 1)
  for i=1:size(tfs, 2)
    T = T*tfs{i};
    cloud = (T*clouds{i+1}')';
    pcshow(cloud(1:ds:end,1:3), colors{mod(i, size(colors, 2))+1}, 'MarkerSize', 1)
  end
  hold off
end
