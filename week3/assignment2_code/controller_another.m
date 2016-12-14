function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;

% FILL IN YOUR CODE HERE

Kvy = 0.10125*6*1;
Kvphi = 0.1025*12*2*10;
Kvz = 0.15875*12*10;
Kpy = .25*6*10;
Kpphi = 56*12*2*10;
Kpz = 20*12*10;

persistent phi_c_dot_prev  phi_c_prev

if isempty(phi_c_prev)
    phi_c_prev = 0;
end

if isempty(phi_c_dot_prev)
    phi_c_dot_prev = 0;
end

phi_c = -(1/params.gravity)*(des_state.acc(1)...
    +Kvy*(des_state.vel(1)-state.vel(1))...
    +Kpy*(des_state.pos(1)-state.pos(1)));

phi_c_dot = (phi_c - phi_c_prev)/0.01;

phi_c_ddot = (phi_c_dot - phi_c_dot_prev)/0.01;

u1 = params.mass*(params.gravity+des_state.acc(2)...
    +Kvz*(des_state.vel(2)-state.vel(2))...
    +Kpz*(des_state.pos(2)-state.pos(2)));

u2 = params.Ixx*(phi_c_ddot+Kvphi*(phi_c_dot-state.omega(1))...
    +Kpphi*(phi_c-state.rot(1)));

phi_c_prev = phi_c;

phi_c_dot_prev = phi_c_dot;


end

