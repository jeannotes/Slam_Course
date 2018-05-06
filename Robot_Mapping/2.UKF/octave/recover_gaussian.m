function [mu, sigma] = recover_gaussian(sigma_points, w_m, w_c)
% This function computes the recovered Gaussian distribution (mu and sigma)
% given the sigma points (size: nx2n+1) and their weights w_m and w_c:
% w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n].
% The weight vectors are each 1x2n+1 in size,
% where n is the dimensionality of the distribution.

% Try to vectorize your operations as much as possible

n = size(sigma_points,1);
mu = zeros(n,1);
sigma = zeros(n,n);

% TODO: compute mu
mu = sigma_points * w_m';

% TODO: compute sigma
sigma = (sigma_points - mu) * diag(w_c) * (sigma_points - mu)';

end
