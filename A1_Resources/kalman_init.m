% The state matrices
dt = 0.01;

% process matrix
A = [1,0,0, dt,0, 0;...
     0,1,0, 0, dt,0;...
     0,0,1, 0, 0, dt;...
     0,0,0, 1, 0, 0;...
     0,0,0, 0, 1, 0;...
     0,0,0, 0, 0, 1];

% noise parameters
% Q
sigma = 0.01;
sigmadot = 0.01;
Q=diag([sigma,sigma,sigma,sigmadot,sigmadot,sigmadot]);
% R
Const_R = 100;
R = Const_R .* eye(3);

% measurement matrix
H = [1,0,0,0,0,0;...
     0,1,0,0,0,0;...
     0,0,1,0,0,0];

% Initial state value
x0 = [0,0,0,0,0,0]';
