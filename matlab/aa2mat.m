function [m] = aa2mat(axis, angle)
  m = [cos(angle) + axis(1)^2*(1-cos(angle)), axis(1)*axis(2)*(1-cos(angle))-axis(3)*sin(angle), axis(1)*axis(3)*(1-cos(angle)) + axis(2)*sin(angle);
  axis(2)*axis(1)*(1-cos(angle))+axis(3)*sin(angle), cos(angle)+axis(2)^2*(1-cos(angle)), axis(2)*axis(3)*(1-cos(angle))-axis(1)*sin(angle);
  axis(3)*axis(1)*(1-cos(angle))-axis(2)*sin(angle), axis(3)*axis(2)*(1-cos(angle)) + axis(1)*sin(angle), cos(angle) + axis(3)^2*(1-cos(angle))];
end
