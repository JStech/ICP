function [] = plot_icp(ref, src, tf, z)
  m = [0 1 0; 0 0 1; 1 0 0];
  if size(ref, 2) == 3
    ref = [ref ones(size(ref, 1), 1)];
  end
  if size(src, 2) == 3
    src = [src ones(size(src, 1), 1)];
  end
  if ~exist('z')
    z = ones(size(src, 1), 1);
  end
  figure;
  hold on
  pcshow(ref(:,1:3)*m, 'blue', 'MarkerSize', 1)
  src_m = (tf * src')';
  pcshow(src_m(find(z>0),1:3)*m, 'green', 'MarkerSize', 1)
  pcshow(src_m(find(z<=0),1:3)*m, 'red', 'MarkerSize', 1)
  hold off
end
