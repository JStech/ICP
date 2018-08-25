function [] = plot_icp(ref, src, tf, ds)
  if ~exist('ds')
    ds = 1;
  end
  n_src = size(src, 1);
  figure;
  hold on
  pcshow(src(1:ds:end,1:3), [1 .5 .5], 'MarkerSize', 1)
  pcshow(ref(1:ds:end,1:3), 'blue', 'MarkerSize', 1)
  src_m = (tf * src')';
  pcshow(src_m(1:ds:end,1:3), 'red', 'MarkerSize', 1)
  hold off
end
