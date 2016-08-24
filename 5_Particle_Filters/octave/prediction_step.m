function particles = prediction_step(particles, u, noise)
% Updates the particles by drawing from the motion model
% Use u.r1, u.t, and u.r2 to access the rotation and translation values
% which have to be pertubated with Gaussian noise.
% The position of the i-th particle is given by the 3D vector
% particles(i).pose which represents (x, y, theta).

% noise parameters
% Assume Gaussian noise in each of the three parameters of the motion model.
% These three parameters may be used as standard deviations for sampling.
r1Noise = noise(1);
transNoise = noise(2);
r2Noise = noise(3);

numParticles = length(particles);

for i = 1:numParticles

  % append the old position to the history of the particle
  particles(i).history{end+1} = particles(i).pose;

  % TODO: sample a new pose for the particle
  mu = particles(i).pose + [u.t*cos(u.r1+particles(i).pose(3)); u.t*sin(u.r1+particles(i).pose(3)); normalize_angle(u.r1+u.r2)];
  mu(3) = normalize_angle(mu(3));
  particles(i).pose(1) = normrnd(mu(1),r1Noise+transNoise+r2Noise);
  particles(i).pose(2) = normrnd(mu(2),r1Noise+transNoise+r2Noise);
  particles(i).pose(3) = normrnd(mu(3),r1Noise+r2Noise);

end

end
