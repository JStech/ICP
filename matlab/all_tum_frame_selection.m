load('../data/tum_all_overlaps.mat');
pairs = {};
c = 1;
for dataset = {'freiburg1_desk', 'freiburg1_desk2', 'freiburg1_floor', ...
  'freiburg1_room', 'freiburg2_360_hemisphere', 'freiburg2_360_kidnap', ...
  'freiburg2_desk', 'freiburg2_large_no_loop', 'freiburg2_large_with_loop', ...
  'freiburg3_long_office_household'}
  o = all_overlaps.(dataset{1});
  for decile=1:9
    population = find(decile/10 <= o & o < (decile+1)/10);
    if (size(population, 1) == 0)
      error('hi')
    end
    sample = randsample(population, 1);
    [i, j] = ind2sub(size(o), sample);
    pairs(c,:) = {dataset{1}, o(i, j), i, j};
    c = c + 1;
  end
end
