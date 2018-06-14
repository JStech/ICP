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
  % make symmetric, apply Gaussian kernel, row-normalize
  hood = hood + hood' .* xor(hood ~= 0, hood' ~= 0);
  hood = spfun(@exp, -hood.^2/(2*params.neighbor_sigma^2));
  row_sums = sum(hood, 2);
  row_sums(row_sums < 1) = 1;
  hood = spdiags(1./row_sums, 0, n, n) * hood;
  data.neighborhoods{1} = hood;

  % sampling
  n = size(hood, 1);
  for level=2:params.pyramid_levels
    new_neighbors = sort(randsample(n, floor(n/2)));

    data.neighborhood_maps{level-1} = new_neighbors;
    h2 = hood(new_neighbors, :);
    h2 = h2 * h2';
    % zero out the diagonal
    h2(1:size(h2, 1)+1:end) = 0;
    % eliminate low-weight edges, for sparseness' sake
    th = 0.1*max(h2(:));
    hood = spfun(@(x) x.*(x>th), h2);
    n = size(hood, 1);
    row_sums = sum(hood, 2);
    row_sums(row_sums < 1) = 1;
    hood = spdiags(1./row_sums, 0, n, n) * hood;
    data.neighborhoods{level} = hood;
  end
end
