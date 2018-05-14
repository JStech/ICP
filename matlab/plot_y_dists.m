max_inlier = 1.05*max([y_in_unal; y_in_al]);
max_outlier = 1.05*max([y_out_unal; y_out_al]);
n_bins = 100;

in_limits = [];
out_limits = [];

subplot(2, 2, 1);
histogram(y_in_unal, n_bins, 'BinLimits', [0, max_outlier]);
title('Unaligned inliers');
in_limits = [in_limits; ylim()];

subplot(2, 2, 2);
histogram(y_in_al, n_bins, 'BinLimits', [0, max_outlier]);
title('Aligned inliers');
in_limits = [in_limits; ylim()];

subplot(2, 2, 3);
histogram(y_out_unal, n_bins, 'BinLimits', [0, max_outlier]);
title('Unaligned outliers');
out_limits = [out_limits; ylim()];

subplot(2, 2, 4);
histogram(y_out_al, n_bins, 'BinLimits', [0, max_outlier]);
title('Aligned outliers');
out_limits = [out_limits; ylim()];

for i=1:2
  subplot(2, 2, i);
  ylim([min(in_limits(:,1)), max(in_limits(:,2))]);
  subplot(2, 2, i+2);
  ylim([min(out_limits(:,1)), max(out_limits(:,2))]);
end

