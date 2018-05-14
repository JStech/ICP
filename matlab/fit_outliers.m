num_samples = 100;
k = 150;

% TODO: is this list right?
distributions = {
  'Burr',             %  1
  'Exponential',      %  2
  'Gamma',            %  3
  'HalfNormal',       %  4
  'Logistic',         %  5
  'Loglogistic',      %  6
  'Lognormal',        %  7
  'Normal',           %  8
  'Poisson',          %  9
  'Rayleigh',         % 10
  'Rician',           % 11
  'tLocationScale',   % 12
  };

results = cell(0);

for i=1:num_samples
  i
  f1 = randi(n-k, 1);
  f2 = f1 + randi(k, 1);
  t1 = pmats{f1};
  t2 = pmats{f2};
  c1 = unproject(getcloud(f1));
  c2 = unproject(getcloud(f2));
  results(i, :) = {f1, f2, zeros(1, length(distributions))};

  [~, ~, ~, y_out_al] = y_dists(c1, c2, inv(inv(t1)*t2));
  [f, x] = ecdf(y_out_al);

  for d=1:length(distributions)
    try
      pd = fitdist(y_out_al, distributions{d});
      ks = max(abs(f - pd.cdf(x)));
      results{i, 3}(d) = ks;
    catch
      disp(sprintf('Skipping %s due to error', distributions{d}))
    end
    pause(1)
  end
end
