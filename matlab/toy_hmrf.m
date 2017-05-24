gt = [-1 -1 -1 -1  1  1  1  1  1  1  1  1  1  1  1  1;
      -1 -1 -1 -1  1  1  1  1  1  1  1  1  1  1  1  1;
      -1 -1 -1 -1  1  1  1  1  1  1  1  1  1  1  1  1;
      -1 -1 -1 -1  1  1  1  1  1  1  1  1  1  1  1  1;
      -1 -1 -1 -1  1  1  1  1  1  1  1  1  1  1  1  1;
      -1 -1 -1 -1  1  1  1  1  1  1  1  1  1  1  1  1;
      -1 -1 -1 -1  1  1  1  1  1  1  1  1  1  1  1  1;
      -1 -1 -1 -1  1  1  1  1  1  1  1  1  1  1  1  1;
      -1 -1 -1 -1  1  1  1  1  1  1  1  1  1  1  1  1;
      -1 -1 -1 -1  1  1  1  1  1  1  1  1  1  1  1  1;
       1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1;
       1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1;
       1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1;
       1  1  1  1  1  1  1  1  1  1  1  1 -1 -1 -1 -1;
       1  1  1  1  1  1  1  1  1  1  1  1 -1 -1 -1 -1;
       1  1  1  1  1  1  1  1  1  1  1  1 -1 -1 -1 -1];
[h w] = size(gt);

mu_in = 3;
sigma_in = 1;
mu_out = 6;
sigma_out = 3;

n_in = sum(sum(gt==1));
n_out = prod(size(gt))-n_in;
y = zeros(size(gt));
y(gt==1) = mu_in + sigma_in*randn(n_in, 1);
y(gt==-1) = mu_out + sigma_out*randn(n_out, 1);

% inference
beta = 2.0;
z = ones(size(y));
if sum(z(:)==1) > 2
  in_mean = mean(reshape(y(z==1), 1, []));
  in_std = std(reshape(y(z==1), 1, []));
else
  in_mean = min(y(:));
  in_std = std(y(:));
end

if sum(z(:)==-1) > 2
  out_mean = mean(reshape(y(z==-1), 1, []));
  out_std = std(reshape(y(z==-1), 1, []));
else
  out_mean = max(y(:));
  out_std = std(y(:));
end

zs = zeros([h, w, 10]);
for i=1:20
  mean_field = ([z(2:end,:); zeros(1, w)] + [zeros(1, w); z(1:end-1,:)] + [z(:,2:end) zeros(h, 1)] + [zeros(h, 1) z(:,1:end-1)])/4;
  r_in = beta*mean_field - log(in_std) - (y - in_mean).^2/(2*in_std.^2);
  r_out = -beta*mean_field - log(out_std) - (y - out_mean).^2/(2*out_std.^2);
  z = 2*exp(r_in) ./ (exp(r_out) + exp(r_in)) - 1;
  zs(:,:,i) = z;
  in_mean = sum(reshape((1+z)/2.*y, [], 1))/sum(reshape((1+z)/2, [], 1));
  in_std = sqrt(sum(reshape((1+z)/2.*y.^2, [], 1))/sum(reshape((1+z)/2, [], 1)));
  out_mean = sum(reshape((1-z)/2.*y, [], 1))/sum(reshape((1-z)/2, [], 1));
  out_std = sqrt(sum(reshape((1-z)/2.*y.^2, [], 1))/sum(reshape((1-z)/2, [], 1)));
  assert(out_mean > in_mean);
end
