function [overlap] = calculate_overlap(c1, c2)
  kdtree1 = KDTreeSearcher(c1);
  kdtree2 = KDTreeSearcher(c2);
  [idx1, dist1] = knnsearch(kdtree1, c2);
  [idx2, dist2] = knnsearch(kdtree2, c2, 'K', 2);

  overlap = sum(dist1 < 8*dist2(:,2), 'omitnan')/sum(~isnan(dist2(:,2)));
end
