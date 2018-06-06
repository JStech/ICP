function [tf, data] = hmrf_icp(ref, src, params)
  data = icp_data();
  % don't need NaNs as placeholders for unstructured data (but we do for grids)
  if strcmp('unstructured', params.neighbor_structure)
    src(any(isnan(src), 2), :) = [];
    ref(any(isnan(ref), 2), :) = [];
  end
  data.src = src;
  data.ref = ref;
  data.em_iters = [];
  data.icp_iters = 0;

  if params.make_animation
    data.anim.clouds = [];
    data.anim.zfields = [];
  end

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
  if params.verbose
    disp('Done building kdtree');
  end

  tf = params.t_init;

  % apply initial transform
  src = (params.t_init * src')';

  if strcmp('grid', params.neighbor_structure)
    grid = true;
    z = ones(params.w, params.h);
  elseif strcmp('unstructured', params.neighbor_structure)
    grid = false;
    z = ones(n_src, 1);
    if params.verbose
      disp('Making neighborhoods');
    end
    data = make_neighborhoods(params, data);
    if params.verbose
      disp('Done making neighborhoods');
    end
  else
    error('parameter "neighbor_structure" must be "grid" or "unstructured"');
  end

  if params.verbose
    disp('Starting ICP iterations');
  end
  for icp_iter=1:params.icp_iter_max
    [idx, y] = knnsearch(kdtree, src);
    if grid
      y = reshape(y, [params.w params.h]);
    end

    if icp_iter==1
      z(find(y > prctile(y, 90))) = -1;
      if params.make_animation
        data.anim.zfields = z;
        data.anim.clouds = make_colored_clouds(src, ref, z);
      end
      % extra EM iterations when we first start
      [z, theta, iters, data] = EM_pyramid(y, z, params.em_iter_max_start, ...
        params, data);
    else
      [z, theta, iters, data] = EM_pyramid(y, z, params.em_iter_max, ...
        params, data);
    end

    % find matches
    matches = [find(z>0) idx(find(z>0))];

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
      disp(sprintf('Iter %d; scale %f, translation %f, rotation %f', icp_iter, scale, dt, dth));
    end

    if dt < params.dt_thresh && dth < params.dth_thresh && abs(scale-1) < params.scale_thresh
      break;
    end
  end
  data.icp_iters = icp_iter;
end

function [z, theta, iters, data] = EM_pyramid(y, z, max_iter, params, data)
  zs = {z};
  ys = {y};
  for p=[2:params.pyramid_levels]
    if strcmp(params.neighbor_structure, 'grid')
      zs{p} = zs{p-1}(1+mod(p, 2):2:end, 1+mod(p, 2):2:end);
      ys{p} = ys{p-1}(1+mod(p, 2):2:end, 1+mod(p, 2):2:end);
    else
      zs{p} = zs{p-1}(data.neighborhood_maps{p-1});
      ys{p} = ys{p-1}(data.neighborhood_maps{p-1});
    end
  end
  for p=[params.pyramid_levels:-1:1]
    data.pyramid_level = p;
    if strcmp(params.neighbor_structure, 'grid')
      params.w = size(zs{p}, 1);
      params.h = size(zs{p}, 2);
    end
    [zs{p} theta iters data] = EM(ys{p}, zs{p}, max_iter, params, data);

    % expand field
    if p>1
      if strcmp(params.neighbor_structure, 'grid')
        zs{p-1} = repelem(zs{p}, 2, 2);
      else
        s = data.neighborhood_maps{p-1};
        n = size(data.neighborhoods{p-1}, 1);
        not_s = 1:n;
        not_s(s) = [];
        zs{p-1}(s) = zs{p};
        zs{p-1}(not_s) = data.neighborhoods{p-1}(not_s, s) * zs{p};
      end
    end
  end
  z = zs{1};
end

function [z, theta, iters, data] = EM(y, z, max_iter, params, data)
  z1 = randi(3, size(z))-2;
  for iters=[1:max_iter]
    z2 = z1;
    z1 = z;
    theta = M_step(z, y);
    z = E_step(y, z, theta, params, data);
    if params.make_animation
      scale = size(data.anim.zfields, 1) / size(z, 1);
      data.anim.zfields = cat(3, data.anim.zfields, repelem(z, scale, scale));
      data.anim.clouds = cat(3, data.anim.clouds, ...
          make_colored_clouds(data.src, data.ref, repelem(z, scale, scale)));
    end
    if all(z(:) .* z1(:) >= 0) || ...
      (iters > 1 && all(z(:) .* z2(:) >= 0))
      break
    end
    if params.verbose && mod(iters, 10)==0
      fprintf('Iters %d: %f\n', iters, sum(z(:)>0)/prod(size(z)));
    end
  end
  data.em_iters = [data.em_iters, iters];
end

function [z] = E_step(y, z, theta, params, data)
  if strcmp('grid', params.neighbor_structure)
    mean_field = [z(2:end,:); zeros(1, params.h)] + [zeros(1, params.h); z(1:end-1,:)] + [z(:,2:end) zeros(params.w, 1)] + [zeros(params.w, 1) z(:,1:end-1)];
  else
    mean_field = data.neighborhoods{data.pyramid_level} * z;
    assert(all(~isnan(mean_field(:))))
  end
  r_in = params.beta*mean_field - 0.5*log(2*pi) - log(theta.in_std) - (y - theta.in_mean).^2/(2*theta.in_std.^2) + params.gamma;
  % fill missing values using just the mean field
  r_in(isnan(r_in)) = params.beta*mean_field(isnan(r_in));
  if strcmp(params.em_outlier_dist, 'normal')
    r_out = -params.beta*mean_field - 0.5*log(2*pi) - log(theta.out_std) - (y - theta.out_mean).^2/(2*theta.out_std.^2) - params.gamma;
  elseif strcmp(params.em_outlier_dist, 'logistic')
    s = sqrt(3)/pi * theta.out_std;
    r_out = -params.beta*mean_field - log(s) - (y - theta.out_mean)./s - 2*log(1 + exp(-(y - theta.out_mean)./s)) - params.gamma;
  else
    error('em_outlier_dist must be "normal" or "logistic"')
  end
  r_out(isnan(r_out)) = params.beta*mean_field(isnan(r_out));
  r_min = min(r_in, r_out);
  assert(all(~isnan(r_in)));
  assert(all(~isnan(r_out)));
  assert(all(~isnan(r_min)));
  z = 2*exp(r_in-r_min) ./ (exp(r_out-r_min) + exp(r_in-r_min)) - 1;
  assert(all(~isnan(z(:))));
end

function [theta] = M_step(z, y)
  theta.in_mean = sum((1+z(:))/2.*y(:), 'omitnan') ...
    /sum((1+z(:))/2, 'omitnan');
  theta.in_std = sqrt(sum((1+z(:))/2.*y(:).^2, 'omitnan') ...
    /sum((1+z(:))/2, 'omitnan') - theta.in_mean.^2);
  theta.out_mean = sum((1-z(:))/2.*y(:), 'omitnan') ...
    /sum((1-z(:))/2, 'omitnan');
  theta.out_std = sqrt(sum((1-z(:))/2.*y(:).^2, 'omitnan') ...
    /sum((1-z(:))/2, 'omitnan') - theta.out_mean.^2);
  assert(~isnan(theta.in_mean));
  assert(~isnan(theta.in_std));
  assert(~isnan(theta.out_mean));
  assert(~isnan(theta.out_std));
end

function [colored_cloud] = make_colored_clouds(src, ref, z)
  n_src = size(src, 1);
  n_ref = size(ref, 1);
  colored_cloud = [[src(:,1:3); ref(:,1:3)] ...
    [repmat(.25-reshape(z, [], 1)/4, [1 2]), ones(n_src, 1);
    ones(n_ref, 1), zeros(n_ref, 2)]];
end
