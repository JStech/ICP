function [tf, iter] = mycp(ref, src, params)
  icp;

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
      % extra EM iteration when we first start
      theta = M_step(z, y);
      z = E_step(y, z, theta, params);
    end

    % update z, find matches
    theta = M_step(z, y);
    z = E_step(y, z, theta, params);
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
  assignin('base', 'matches_mycp', all_matches);
end

function [z] = E_step(y, z, theta, params)
  mean_field = [z(2:end,:); zeros(1, params.h)] + [zeros(1, params.h); z(1:end-1,:)] + [z(:,2:end) zeros(params.w, 1)] + [zeros(params.w, 1) z(:,1:end-1)];
  assignin('base', 'db_params', params);
  assignin('base', 'db_mean_field', mean_field);
  assignin('base', 'db_theta', theta);
  assignin('base', 'db_y', y);
  r_in = params.beta*mean_field - log(theta.in_std) - (y - theta.in_mean).^2/(2*theta.in_std.^2) + params.gamma;
  r_in(isnan(r_in)) = params.beta*mean_field(isnan(r_in));
  r_out = -params.beta*mean_field - log(theta.out_std) - (y - theta.out_mean).^2/(2*theta.out_std.^2) - params.gamma;
  r_out(isnan(r_out)) = params.beta*mean_field(isnan(r_out));
  z = 2*exp(r_in) ./ (exp(r_out) + exp(r_in)) - 1;
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
