function anim_zs(zs)
  for i = 1:size(zs, 3)
    imagesc(zs(:,:,i));
    waitforbuttonpress
  end
end
