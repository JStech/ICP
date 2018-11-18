function [r] = calculate_rmse(c1, c2, overlap)
  if size(c1, 2) == 4
    c1 = c1(:,1:3)./c1(:,4);
  end
  if size(c2, 2) == 4
    c2 = c2(:,1:3)./c2(:,4);
  end
  kdtree = KDTreeSearcher(c1);
  [~, rs] = knnsearch(kdtree, c2(overlap,:));
  n = sum(overlap);
  r = sqrt(sum(rs.^2)/n);
end
