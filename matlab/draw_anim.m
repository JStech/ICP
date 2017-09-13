xlim = [min(min(anim.clouds(:,1,:))) max(max(anim.clouds(:,1,:)))];
ylim = [min(min(anim.clouds(:,2,:))) max(max(anim.clouds(:,2,:)))];
zlim = [min(min(anim.clouds(:,3,:))) max(max(anim.clouds(:,3,:)))];

p = pcplayer(xlim, ylim, zlim);
for i=1:size(anim.clouds, 3)
  view(p, anim.clouds(:,1:3,i), anim.clouds(:,4:6,i));
  pause(0.03);
end
