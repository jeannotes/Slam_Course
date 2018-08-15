function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)
theta_hat = mu(3);
dmu = zeros(size(sigma,1),1);
dmu(1) = u.t * cos(theta_hat + u.r1);
dmu(2) = u.t * sin(theta_hat + u.r1);
dmu(3) = u.r1 + u.r2;
mu = mu + dmu;
mu(3) = normalize_angle(mu(3));
 
% TODO: Compute the 3x3 Jacobian Gx of the motion model
Gx = [1 0 -u.t * sin(theta_hat + u.r1);
      0 1  u.t * cos(theta_hat + u.r1);
      0 0  1];

% TODO: Construct the full Jacobian G
G = eye(size(sigma,1));
G(1:3,1:3) = Gx;
% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Compute the predicted sigma after incorporating the motion
sigma = G * sigma * transpose(G) + R;

end
