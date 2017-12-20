function [out_cloud] = downsample(cloud, w, h, s)
  out_cloud = reshape(cloud, w, h, []);
  out_cloud = out_cloud(1:s:end, 1:s:end, :);
  out_cloud = reshape(out_cloud, floor(w/s)*floor(h/s), []);
end
