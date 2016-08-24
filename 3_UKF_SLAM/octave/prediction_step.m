function [mu, sigma, sigma_points] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model.
% mu: state vector containing robot pose and poses of landmarks obeserved so far
% Current robot pose = mu(1:3)
% Note that the landmark poses in mu are stacked in the order by which they were observed
% sigma: the covariance matrix of the system.
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% For computing lambda.
global scale;

% Compute sigma points
sigma_points = compute_sigma_points(mu, sigma);

% Dimensionality
n = length(mu);
% lambda
lambda = scale - n;

% TODO: Transform all sigma points according to the odometry command
% Remember to vectorize your operations and normalize angles
% Tip: the function normalize_angle also works on a vector (row) of angles
for i=1:2*n+1
    sigma_points(1:3,i) = sigma_points(1:3,i) + [u.t*cos(sigma_points(3,i) + u.r1); u.t*sin(sigma_points(3,i) + u.r1); u.r1 + u.r2];
    sigma_points(3,i) = normalize_angle(sigma_points(3,i));
endfor
% Computing the weights for recovering the mean
wm = [lambda/scale, repmat(1/(2*scale),1,2*n)];
wc = wm;

% -------- My initialization ----------- %
xbar = 0;
ybar = 0;
mu = zeros(n,1);
sigma = zeros(n);
% TODO: recover mu.
% Be careful when computing the robot's orientation (sum up the sines and
% cosines and recover the 'average' angle via atan2)
for i=1:2*n+1
    mu = mu + wm(i)*sigma_points(:,i);
    xbar = xbar + wm(i)*cos(sigma_points(3,i));
    ybar = ybar + wm(i)*sin(sigma_points(3,i));    
endfor
mu(3) = atan2(ybar,xbar);
mu(3) = normalize_angle(mu(3));

% TODO: Recover sigma. Again, normalize the angular difference
for i=1:2*n+1
    tmp = sigma_points(:,i) - mu;
    tmp(3) = normalize_angle(tmp(3));
    sigma = sigma + wc(i)*tmp*tmp';
endfor

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% TODO: Add motion noise to sigma
sigma = sigma + R;

end
