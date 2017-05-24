function [] = cmp_matches(matches1, matches2)
  z1 = zeros(640, 480);
  z1(matches1(:,1)) = 1;
  subplot(1, 3, 1); imagesc(z1); colorbar;
  z2 = zeros(640, 480);
  z2(matches2(:,1)) = 1;
  subplot(1, 3, 3); imagesc(z2); colorbar;
  subplot(1, 3, 2); imagesc(z1+z2 - 2*z1.*z2); colorbar;
end
