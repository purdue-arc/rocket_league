% Use MATLAB sybolic eq toolbox to solve for the equation used to
% linearize the bicycle model

syms x y theta v beta DELTA_T CAR_LENGTH;  % input state

% discretized bicycle model
next_x = x+ v*cos(theta+beta)*DELTA_T;
next_y = y+ v*sin(theta+beta)*DELTA_T;

radius = CAR_LENGTH/(2*sin(beta));
next_theta = theta + radius*v*DELTA_T;

% compute jacobian
state = [x; y; theta; v; beta];
next_state = [next_x; next_y; next_theta; v; beta]
F = jacobian(next_state, state)

