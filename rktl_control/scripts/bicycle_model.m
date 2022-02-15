% Use MATLAB sybolic eq toolbox to solve for the equation used to
% linearize the bicycle model

syms DELTA_T CAR_LENGTH; % constants
syms x y theta v_rear psi;  % input state

% intermediate calculations
beta = atan(tan(psi)/2);
v_body = v_rear/cos(beta);
curvature = 2*sin(beta)/CAR_LENGTH;

% next state calculations
next_x = x + v_body*cos(theta+beta)*DELTA_T;
next_y = y + v_body*sin(theta+beta)*DELTA_T;
next_theta = theta + curvature*v_body*DELTA_T;

% compute jacobian
state = [x; y; theta; v_rear; psi];
next_state = [next_x; next_y; next_theta; v_rear; psi]
F = jacobian(next_state, state)

