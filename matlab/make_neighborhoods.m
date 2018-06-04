function data = make_neighborhoods(params, data)
  n = size(data.src, 1);
  data.neighborhoods = cell(params.pyramid_levels, 1);
  data.neighborhood_maps = cell(params.pyramid_levels-1, 1);

  kdtree = KDTreeSearcher(data.src);
  [i, dist] = knnsearch(kdtree, data.src, 'K', params.max_neighbors+1);
  i = reshape(i(:, 2:end), 1, []);
  dist = reshape(dist(:, 2:end), 1, []);

  % build base neighborhood
  j = repmat([1:n], 1, params.max_neighbors);
  hood = sparse(i, j, dist, n, n);
  % make symmetric
  hood = hood + hood' .* xor(hood ~= 0, hood' ~= 0);
  data.neighborhoods{1} = hood;

  % apply Gaussian kernel
  hood = spfun(@exp, -hood.^2/(2*params.neighbor_sigma^2));

  % sampling
  for level=2:params.pyramid_levels
    n = size(hood, 1);
    next_sample = zeros(n, 1);
    disp(sprintf('sampling %d', level));
    while any(next_sample == 0)
      p = find(next_sample == 0);
      s = randsample(p, ceil(length(p)/10));
      next_sample(s) = 1;
      [~, j] = find(hood(s, :));
      next_sample(j) = -1;
    end
    disp(sprintf('done sampling %d', level));

    new_neighbors = find(next_sample > 0);
    data.neighborhood_maps{level-1} = new_neighbors;
    h2 = hood(new_neighbors, :);
    h2 = h2 * h2';
    % zero out the diagonal
    h2(1:size(h2, 1)+1:end) = 0;
    % eliminate low-weight edges, for sparseness' sake
    th = 0.1*max(h2(:));
    h2(h2(:) < th) = 0;
    hood = h2;
    data.neighborhoods{level} = hood;
  end
end
