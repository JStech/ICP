function [r] = calculate_rmse(c1, c2, overlap)
  if size(c1, 2) == 4
    c1 = project(c1);
  end
  if size(c2, 2) == 4
    c2 = project(c2);
  end
  kdtree = KDTreeSearcher(c1);
  [~, rs] = knnsearch(kdtree, c2(overlap,:));
  n = sum(overlap);
  r = sqrt(sum(rs.^2)/n);
end
