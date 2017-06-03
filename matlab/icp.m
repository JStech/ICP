function [tf, iter, all_matches] = icp(ref, src, params)
  assignin('caller', 'plot_icp', @(ref, src, tf, ds) plot_icp(ref, src, tf, ds));
  if nargin==0
    assignin('caller', 'localize', @(ref, src, do_scale) localize(ref, src, do_scale));
    assignin('caller', 'mat2aa', @(m) mat2aa(m));
    assignin('caller', 'aa2mat', @(axis, angle) aa2mat(axis, angle));
    tf = 0;
    matched = 0;
    return
  end

  if strcmp(params.mode, 'dynamic')
    Dmax = 20*params.D;
  end

  % drop NaNs
  ref = ref(find(~isnan(ref(:,1))), :);
  src = src(find(~isnan(src(:,1))), :);

  % we'll work with homogenous coords
  n_ref = size(ref, 1);
  n_src = size(src, 1);
  assert(size(ref, 2) == 4);
  assert(size(src, 2) == 4);

  % build kdtree
  %disp('Building kdtree');
  kdtree = KDTreeSearcher(ref);

  tf = params.t_init;
  src = (tf * src')';

  all_matches = cell(0);
  for iter=1:params.iter_max
    [idx, dist] = knnsearch(kdtree, src);

    switch params.mode
      case 'all'
        matches = [[1:n_src]' idx];
      case 'pct'
        [~, matches] = sort(dist);
        matches = matches(1:floor(0.9*n_src));
        matches = [matches idx(matches)];
      case 'sigma'
        mu = mean(dist);
        sigma = std(dist);
        Dmax = mu + 2.5*sigma;
        matches = find(dist < Dmax);
        matches = [matches idx(matches)];
      case 'dynamic'
        matches = find(dist < Dmax);
        mu = mean(dist(matches));
        sigma = std(dist(matches));

        if mu < params.D
          Dmax = mu + 3*sigma;
        elseif mu < 3*params.D
          Dmax = mu + 2*sigma;
        elseif mu < 6*params.D
          Dmax = mu + sigma;
        else
          Dmax = choose_xi(dist, idx);
        end

        matches = find(dist < Dmax);
        matches = [matches idx(matches)];
    end
    all_matches{iter} = matches;

    % calculate transformation
    Tmat = localize(ref(matches(:,2),:), src(matches(:,1),:), params.do_scale);
    tf = Tmat * tf;

    % apply to source cloud
    src = (Tmat * src')';

    % stopping criteria
    scale = norm(Tmat(1:3,1));
    dt = norm(Tmat(1:3,4));
    dth = acos((trace(Tmat(1:3,1:3))/scale - 1)/2);

    %disp(sprintf('Iter %d; scale %f, translation %f, rotation %f', iter, scale, dt, dth));

    if dt < params.dt_thresh && dth < params.dth_thresh && ...
          abs(scale-1) < params.scale_thresh
      break;
    end
  end
end

function [Dmax] = choose_xi(dist, idx)
  [h, e] = histcounts(dist, 20);
  [m, i] = max(h);
  for ii=1:20
    if i+ii == 20
      Dmax = e(20);
      break
    end
    if h(i+ii-1) > h(i+ii) && h(i+ii) < h(i+ii+1)
      Dmax = e(i+ii)
      break;
    end
  end
end

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

function [axis, angle] = mat2aa(m)
  angle = acos((m(1,1) + m(2,2) + m(3,3) - 1)/2);
  axis = [m(3, 2) - m(2, 3); m(1, 3) - m(3, 1); m(2, 1) - m(1, 2)];
  axis = axis ./ norm(axis);
end

function [m] = aa2mat(axis, angle)
  m = [cos(angle) + axis(1)^2*(1-cos(angle)), axis(1)*axis(2)*(1-cos(angle))-axis(3)*sin(angle), axis(1)*axis(3)*(1-cos(angle)) + axis(2)*sin(angle);
  axis(2)*axis(1)*(1-cos(angle))+axis(3)*sin(angle), cos(angle)+axis(2)^2*(1-cos(angle)), axis(2)*axis(3)*(1-cos(angle))-axis(1)*sin(angle);
  axis(3)*axis(1)*(1-cos(angle))-axis(2)*sin(angle), axis(3)*axis(2)*(1-cos(angle)) + axis(1)*sin(angle), cos(angle) + axis(3)^2*(1-cos(angle))];
end
