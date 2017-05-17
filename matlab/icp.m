function [tf, matched] = icp(ref, src, D, t_init, iter_max)
  assignin('base', 'plot_icp', @(ref, src, tf) plot_icp(ref, src, tf));
  if nargin==0
    assignin('base', 'localize', @(ref, src) localize(ref, src));
    tf = 0;
    matched = 0;
    return
  end

  Dmax = 20*D;
  dt_thresh = 0.05;
  dth_thresh = 0.001;
  scale_thresh = 0.001;

  % we'll work with homogenous coords
  n_ref = size(ref, 1);
  n_src = size(src, 1);
  ref = [ref ones(n_ref, 1)];
  src = [src ones(n_src, 1)];

  % find Is-a-Numbers
  refIaN = find(~isnan(ref(:,1)));
  srcIaN = find(~isnan(src(:,1)));

  % build kdtree
  disp('Building kdtree');
  kdtree = KDTreeSearcher(ref);

  % initialize transform
  tf = eye(4);
  if nargin > 3
    % apply initial transform
    src = t_init * src;
    tf = t_init;
  end

  if nargin < 5
    iter_max = 30;
  end

  for iter=1:iter_max
    [idx, dist] = knnsearch(kdtree, src);

    matches = find(dist < Dmax);
    mu = mean(dist(matches));
    sigma = std(dist(matches));

    if mu < D
      Dmax = mu + 3*sigma;
    elseif mu < 3*D
      Dmax = mu + 2*sigma;
    elseif mu < 6*D
      Dmax = mu + sigma;
    else
      Dmax = choose_xi(dist, idx);
    end

    matches = find(dist < Dmax);
    matches = [matches idx(matches)];
    assert(size(matches, 2) == 2);

    % calculate transformation
    Tmat = localize(ref(matches(:,2),:), src(matches(:,1),:));
    tf = Tmat * tf;

    % apply to source cloud
    src = (Tmat * src')';

    % stopping criteria
    scale = norm(Tmat(1:3,1));
    dt = norm(Tmat(1:3,4));
    dth = acos((trace(Tmat(1:3,1:3))/scale - 1)/2);

    disp(sprintf('Iter %d; scale %f, translation %f, rotation %f', iter, scale, dt, dth));

    if dt < dt_thresh && dth < dth_thresh && abs(scale-1) < scale_thresh
      break;
    end
  end
  matched = matches;
end

function [Dmax] = choose_xi(dist, idx)
  Dmax = 100;
end

function [tf] = localize(ref, src)
  assert(size(ref, 2) == 4);
  assert(size(src, 2) == 4);
  assert(~any(isnan(ref(:))));
  assert(~any(isnan(src(:))));

  % project
  ref = ref(:,1:3)./ref(:,4);
  src = src(:,1:3)./src(:,4);

  ref_centroid = mean(ref);
  src_centroid = mean(src);

  scale = mean(sqrt(sum((src - src_centroid).^2, 2)./...
  sum((ref - ref_centroid).^2, 2)));
  assert(all(size(scale) == [1 1]));

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

function [] = plot_icp(ref, src, tf)
  n_src = size(src, 1);
  figure;
  hold on
  pcshow(src(:,1:3), 'red')
  pcshow(ref(:,1:3), 'blue')
  src_m = tf * [src ones(n_src, 1)]';
  src_m = src_m(1:3,:)';
  pcshow(src_m, 'green')
  hold off
end
