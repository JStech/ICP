function [r] = extract_res(res, k)
r = zeros(size(res));
for i = 1:size(res,1)
  for j = 1:size(res,2)
    r(i,j) = res{i,j}(k);
  end
end
end
