function [sigma] = calc_neighbor_sigma(cloud)
kdtree = KDTreeSearcher(cloud);
[~, dist] = knnsearch(kdtree, cloud, 'K', 2);
sigma = mean(dist(:,2))/2;
end
