function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
F = 0;

% Moment
M = zeros(3,1);

k_d = [1;1;1];
k_p = [100;100;800];
k_drot = [1;1;1];
k_prot =[160;160;160];

r_dotdot_des = des_state.acc + k_d.*(des_state.vel - state.vel) + k_p.*(des_state.pos - state.pos);
des_state.phi = (1/params.gravity)*(r_dotdot_des(1)*sin(des_state.yaw) - r_dotdot_des(2)*cos(des_state.yaw));
des_state.theta = (1/params.gravity)*(r_dotdot_des(1)*cos(des_state.yaw) + r_dotdot_des(2)*sin(des_state.yaw));
des_state.rot = [des_state.phi;des_state.theta;des_state.yaw];
des_state.omega = [0;0;des_state.yawdot];

F = params.mass*(params.gravity + r_dotdot_des(3));   % also known as u1

M = k_prot.*(des_state.rot - state.rot) + k_drot.*(des_state.omega - state.omega); % also known as u2

% =================== Your code ends here ===================


end
