function [tf, iter, iter_start] = mycp(ref, src, params)
  icp;

  if params.make_animation
    anim.clouds = [];
    anim.zfields = [];
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

  all_matches = cell(0);
  matches = 0;
  z = ones(params.w, params.h);
  for iter=1:params.iter_max
    [idx, y] = knnsearch(kdtree, src);
    y = reshape(y, [params.w params.h]);

    if iter==1
      z(find(y > prctile(y, 90))) = -1;
      if params.make_animation
        anim.zfields = z;
        anim.clouds = make_colored_clouds(src, ref, z);
      end
      % extra EM iterations when we first start
      start_zs = zeros(params.w, params.h, params.iter_max_start+1);
      z_1 = ones(params.w, params.h);
      for iter_start=1:params.iter_max_start
        start_zs(:,:,iter_start) = z;
        z_2 = z_1;
        z_1 = z;
        theta = M_step(z, y);
        z = E_step(y, z, theta, params);
        if params.make_animation
          anim.zfields = cat(3, anim.zfields, z);
          anim.clouds = cat(3, anim.clouds, make_colored_clouds(src, ref, z));
        end
        % second condition is to catch oscillating field
        if all(z(:) .* z_1(:) >= 0) || ...
          (iter_start > 1 && all(z(:) .* z_2(:) >= 0))
          break
        end
        if params.verbose && mod(iter_start, 10)==0
          fprintf('Pre-iters %d: %f\n', iter_start, sum(z(:)>0)/prod(size(z)))
        end
      end
      start_zs(:,:,iter_start+1) = z;
      start_zs(:,:,iter_start+2) = y;
      assignin('base', 'start_zs', start_zs)
      if params.verbose
        fprintf('Pre-iters %d: %f\n', iter_start, sum(z(:)>0)/prod(size(z)))
      end
    end

    % update z, find matches
    for inner_iter=1:params.iter_max_inner
      theta = M_step(z, y);
      z_2 = z_1;
      z_1 = z;
      z = E_step(y, z, theta, params);
      if params.make_animation
        anim.zfields = cat(3, anim.zfields, z);
        anim.clouds = cat(3, anim.clouds, make_colored_clouds(src, ref, z));
      end
      if all(z(:) .* z_1(:) >= 0) || ...
        all(z(:) .* z_2(:) >= 0)
        break
      end
    end
    matches = [find(z>0) idx(find(z>0))];

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

    if dt < params.dt_thresh && dth < params.dth_thresh && abs(scale-1) < params.scale_thresh
      break;
    end
  end
  assignin('base', 'z_final', z);
  if params.save_matches
    assignin('base', 'matches_mycp', all_matches);
  end
  if params.make_animation
    assignin('base', 'anim', anim);
  end
end

function [z] = E_step(y, z, theta, params)
  pause(1)
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
  assert(~isnan(theta.in_mean))
  assert(~isnan(theta.in_std))
  assert(~isnan(theta.out_mean))
  assert(~isnan(theta.out_std))
end

function [colored_cloud] = make_colored_clouds(src, ref, z)
  n_src = size(src, 1);
  n_ref = size(ref, 1);
  colored_cloud = [[src(:,1:3); ref(:,1:3)] ...
    [repmat(.25-reshape(z, [], 1)/4, [1 2]), ones(n_src, 1);
    ones(n_ref, 1), zeros(n_ref, 2)]];
end
