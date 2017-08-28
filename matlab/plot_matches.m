function [] = plot_matches(matches, params)
  for m = matches
    subplot(1, 2, 1);
    z = zeros(params.w, params.h);
    z(m{1}(:,1)) = 1;
    imagesc(z);
    subplot(1, 2, 2);
    z = zeros(params.w, params.h);
    z(m{1}(:,2)) = 1;
    imagesc(z);
    colorbar;
    waitforbuttonpress
  end
end
