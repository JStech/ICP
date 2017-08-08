function [] = plot_matches(matches)
  for m = matches
    subplot(1, 2, 1);
    z = zeros(640, 480);
    z(m{1}(:,1)) = 1;
    imagesc(z);
    subplot(1, 2, 2);
    z = zeros(640, 480);
    z(m{1}(:,2)) = 1;
    imagesc(z);
    colorbar;
    waitforbuttonpress
  end
end
