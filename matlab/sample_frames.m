n = size(overlaps, 1);
step = 10;

ols = [0 0 0];
for i=1:step:n
  for j=1:step:n
    ols = [ols; overlaps(i, j) i j];
  end
end

[~, sorted_order] = sort(ols(:,1));

selected = [];
pairs_per_decile = 10;
for decile=0.1:0.1:1
  selected = [selected
  datasample(find(decile-0.1 < ols(:,1) & ols(:,1) < decile), pairs_per_decile)];
end

selected = ols(selected, :);
