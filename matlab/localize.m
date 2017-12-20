function [tf] = localize(ref, src, do_scale)
  if nargin==2
    do_scale = false;
  end
  assert(size(ref, 2) == 4);
  assert(size(src, 2) == 4);
  assert(~any(isnan(ref(:))));
  assert(~any(isnan(src(:))));

  % project
  ref = ref(:,1:3)./ref(:,4);
  src = src(:,1:3)./src(:,4);

  assert(all(~isinf(ref(:))));
  assert(all(~isinf(src(:))));

  ref_centroid = mean(ref);
  src_centroid = mean(src);

  if do_scale
    scale = mean(sqrt(sum((src - src_centroid).^2, 2)./...
    sum((ref - ref_centroid).^2, 2)));
  else
    scale = 1.;
  end

  M = ((src - src_centroid)/scale)' * (ref - ref_centroid);
  [u s v] = svd(M);
  R = eye(4);
  R(1:3, 1:3) = (u * v')'/scale;
  T1 = eye(4);
  T2 = eye(4);
  T1(1:3, 4) = -src_centroid;
  T2(1:3, 4) = ref_centroid;
  tf = T2 * R * T1;
end
