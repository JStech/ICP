function [tf, iter, all_matches] = icp(ref, src, params)
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
  if params.verbose
    disp('Building kdtree');
  end
  kdtree = KDTreeSearcher(ref);

  tf = params.t_init;
  src = (tf * src')';

  all_matches = cell(0);
  for iter=1:params.icp_iter_max
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
      case 'x84'
        med = median(dist);
        mad = median(abs(dist-med));
        Dmax = med + 5.2*mad;
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
      otherwise
        error(['Bad mode ' params.mode])
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

    if params.verbose
      disp(sprintf('Iter %d; scale %f, translation %f, rotation %f', iter, scale, dt, dth));
    end

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
