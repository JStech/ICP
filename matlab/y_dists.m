function [y_in_unal y_out_unal y_in_al y_out_al] = y_dists(c1, c2, t)
[~, overlap, y_al, kdtree1] = calculate_overlap(c1, (t*c2')');
if size(c2, 2) == 4
  c2 = c2(:,1:3)./c2(:,4);
end
[~, y_unal] = knnsearch(kdtree1, c2);
y_in_unal = double(y_unal(find(overlap==1)));
y_out_unal = double(y_unal(find(overlap==0)));
y_in_al = double(y_al(find(overlap==1)));
y_out_al = double(y_al(find(overlap==0)));
end
