function [overlap_fraction overlap] = calculate_overlap(c1, c2)
  if size(c1, 2) == 4
    c1 = c1(:,1:3)./c1(:,4);
  end
  if size(c2, 2) == 4
    c2 = c2(:,1:3)./c2(:,4);
  end
  kdtree1 = KDTreeSearcher(c1);
  kdtree2 = KDTreeSearcher(c2);
  [idx1, dist1] = knnsearch(kdtree1, c2);
  [idx2, dist2] = knnsearch(kdtree2, c2, 'K', 2);

  overlap = (dist1 < 6*dist2(:,2));
  overlap_fraction = sum(overlap, 'omitnan')/sum(~isnan(dist2(:,2)));
end
