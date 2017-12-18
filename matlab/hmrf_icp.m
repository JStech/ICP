function [tf, data] = hmrf_icp(ref, src, params)
  icp;
  data = icp_data();
  data.params = params;
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

  tf = params.t_init;

  % apply initial transform
  src = (params.t_init * src')';

  z = ones(params.w, params.h);
  for icp_iter=1:params.icp_iter_max
    [idx, y] = knnsearch(kdtree, src);
    y = reshape(y, [params.w params.h]);

    if icp_iter==1
      z(find(y > prctile(y, 90))) = -1;
      if params.make_animation
        data.anim.zfields = z;
        data.anim.clouds = make_colored_clouds(src, ref, z);
      end
      % extra EM iterations when we first start
      [z, theta, iters, data] = EM(y, z, params.em_iter_max_start, data);
      % EM_pyramid(y, z, params.em_iter_max_start, 3, data);
    else
      [z, theta, iters, data] = EM(y, z, params.em_iter_max, data);
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

function [z] = E_step(y, z, theta, params)
  mean_field = [z(2:end,:); zeros(1, params.h)] + [zeros(1, params.h); z(1:end-1,:)] + [z(:,2:end) zeros(params.w, 1)] + [zeros(params.w, 1) z(:,1:end-1)];
  r_in = params.beta*mean_field - log(theta.in_std) - (y - theta.in_mean).^2/(2*theta.in_std.^2) + params.gamma;
  r_in(isnan(r_in)) = params.beta*mean_field(isnan(r_in));
  r_out = -params.beta*mean_field - log(theta.out_std) - (y - theta.out_mean).^2/(2*theta.out_std.^2) - params.gamma;
  r_out(isnan(r_out)) = params.beta*mean_field(isnan(r_out));
  r_min = min(r_in, r_out);
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

function [z, theta, iters, data] = EM_pyramid(y, z, max_iter, pyramid_levels, data)
  zs = {z};
  ys = {y};
  for p=[2:pyramid_levels]
    zs{p} = zs{p-1}(1+mod(p, 2):2:end,1:2:end);
    ys{p} = ys{p-1}(1+mod(p, 2):2:end,1:2:end);
  end
  for p=[pyramid_levels:-1:1]
    [zs{p} theta iters data] = EM(ys{p}, zs{p}, max_iter, data);
  end
end

function [z, theta, iters, data] = EM(y, z, max_iter, data)
  z1 = randi(3, data.params.w, data.params.h)-2;
  for iters=[1:max_iter]
    z2 = z1;
    z1 = z;
    theta = M_step(z, y);
    z = E_step(y, z, theta, data.params);
    if data.params.make_animation
      data.anim.zfields = cat(3, data.anim.zfields, z);
      data.anim.clouds = cat(3, data.anim.clouds, ...
          make_colored_clouds(data.src, data.ref, z));
    end
    if all(z(:) .* z1(:) >= 0) || ...
      (iters > 1 && all(z(:) .* z2(:) >= 0))
      break
    end
    if data.params.verbose && mod(iters, 10)==0
      fprintf('Iters %d: %f\n', iters, sum(z(:)>0)/prod(size(z)));
    end
  end
  data.em_iters = [data.em_iters, iters];
end

function [colored_cloud] = make_colored_clouds(src, ref, z)
  n_src = size(src, 1);
  n_ref = size(ref, 1);
  colored_cloud = [[src(:,1:3); ref(:,1:3)] ...
    [repmat(.25-reshape(z, [], 1)/4, [1 2]), ones(n_src, 1);
    ones(n_ref, 1), zeros(n_ref, 2)]];
end
