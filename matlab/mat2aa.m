function [axis, angle] = mat2aa(m)
  angle = acos((m(1,1) + m(2,2) + m(3,3) - 1)/2);
  axis = [m(3, 2) - m(2, 3); m(1, 3) - m(3, 1); m(2, 1) - m(1, 2)];
  axis = axis ./ norm(axis);
end
