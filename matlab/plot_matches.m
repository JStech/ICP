function [] = plot_matches(matches)
  for m = matches
    z = zeros(640, 480);
    z(m{1}(:,1)) = 1;
    imagesc(z);
    colorbar;
    waitforbuttonpress
  end
end
