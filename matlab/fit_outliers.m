for dset={'shark' 'desk' 'room'}
  dataset = dset{1};
  load_data; load_poses;
  num_samples = 100;
  k = 150;

  % TODO: is this list right?
  distributions = {
    'Gamma',            % 1
    'Logistic',         % 2
    'Loglogistic',      % 3
    'Lognormal',        % 4
    'Normal',           % 5
    'Rayleigh',         % 6
    'tLocationScale',   % 7
    };

  results = cell(0);

  for i=1:num_samples
    disp(sprintf('%s %d', dataset, i));
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
      %pause(3)
    end
  end
  all_results.(dataset) = results;
end
