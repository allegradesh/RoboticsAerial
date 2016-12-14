function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

%persistent waypoints0 traj_time d0
%if nargin > 2    % gives waypoints
%    d = waypoints(:,2:end) - waypoints(:,1:end-1);
%    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);  % 2*distance between 2 waypoints
%    traj_time = [0, cumsum(d0)];   % traj_time for every piece pi, here for constant velocity of 0.5m/s 
%    waypoints0 = waypoints;
%    
%else             % only gives t and current state
%    if(t > traj_time(end))
%        t = traj_time(end);
%    end
%    t_index = find(traj_time >= t,1);
%
%    if(t_index > 1)
%        t = t - traj_time(t_index-1);
%    end
%    if(t == 0)
%        desired_state.pos = waypoints0(:,1);
%    else
%        scale = t/d0(t_index-1);
%        desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%    end
%    desired_state.vel = zeros(3,1);
%    desired_state.acc = zeros(3,1);
%    desired_state.yaw = 0;
%    desired_state.yawdot = 0;
%end
%

%% Fill in your code here
% Minimum Snap trajectory algorithm
% N-1 smooth piecewise 7th order polynomials for N waypoints
% Pi = ai1 + ai2*scale + ai3*scale^2 + ai4*scale^3 + ai5*scale^4 + ai6*scale^5 + ai7*scale^6 + ai8*scale^7
% i = 1:n,  scale = (t - Si-1)/Ti
persistent waypoints0 traj_time coeff d0
if nargin > 2    % gives waypoints
    % setup trajectory segment times
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);  % 2*distance between 2 waypoints
    traj_time = [0, cumsum(d0)];   % traj_time for w0 - wi, during every piece pi, assume for constant velocity of 0.5m/s    
    waypoints0 = waypoints;
    
    % calculate coeff  N(i = 1:n)*8(a0...a7)*3(x,y,z) matrix
    % the other part of pi(t)  (without the coefficient) 
    n = size(waypoints, 2) - 1; 
    pc = [ 1  1  1   1     1       1         1         1              % coeff for k=0 (derivative)          
           0  1  2   3     4       5         6         7              % coeff for k=1 (derivative) 
           0  0  2*1 3*2   4*3     5*4       6*5       7*6            % coeff for k=2 (derivative) 
           0  0  0   3*2*1 4*3*2   5*4*3     6*5*4     7*6*5          % coeff for k=3 (derivative) 
           0  0  0   0     4*3*2*1 5*4*3*2   6*5*4*3   7*6*5*4        % coeff for k=4 (derivative) 
           0  0  0   0     0       5*4*3*2*1 6*5*4*3*2 7*6*5*4*3      % coeff for k=5 (derivative) 
           0  0  0   0     0       0         6*5*4*3*2 7*6*5*4*3*2 ]; % coeff for k=6 (derivative)
   
    A = zeros(8*n,8*n);       
    for i = 1:n
        % Constraint 1 ==> Pi(Si-1) = wi-1 for all i=1:n   (2n constraints)
        A(i,(i-1)*8+1:i*8) = [1 0 0 0 0 0 0 0];
        % Constraint 2 ==> Pi(Si) = wi for all i=1:n   (2n constraints)
        A(n+i,(i-1)*8+1:i*8) = pc(1,:);
        % Constraint 5 ==> Pi-1_k(Si) = Pi_k(Si) for all k=1..6  (6n-6 constraints)
        if i ~= n
            A(2*n+i,(i-1)*8+1:(i-1)*8+16) = [pc(2,:) -[0 1 0 0 0 0 0 0]];       % k = 1  
            A(3*n+i,(i-1)*8+1:(i-1)*8+16) = [pc(3,:) -[0 0 2 0 0 0 0 0]];       % k = 2
            A(4*n+i,(i-1)*8+1:(i-1)*8+16) = [pc(4,:) -[0 0 0 3*2 0 0 0 0]];       % k = 3
            A(5*n+i,(i-1)*8+1:(i-1)*8+16) = [pc(5,:) -[0 0 0 0 4*3*2 0 0 0]];       % k = 4
            A(6*n+i,(i-1)*8+1:(i-1)*8+16) = [pc(6,:) -[0 0 0 0 0 5*4*3*2 0 0]];       % k = 5
            A(7*n+i,(i-1)*8+1:(i-1)*8+16) = [pc(7,:) -[0 0 0 0 0 0 6*5*4*3*2 0]];       % k = 6  
        end
    end
    % Constraint 3 ==> P1_k(S0) = 0 for all k=1..3 (derivative) (3 constraints)
    % Constraint 4 ==> Pn_k(Sn) = 0 for all k=1..3 (derivative) (3 constraints)
    % replace those created by constraint 5
    A(2*n+n,1:8) = [0 1 0 0 0 0 0 0];   % k = 1, i = 1
    A(3*n+n,(n-1)*8+1:n*8) = pc(2,:);   % k = 1, i = n
    A(4*n+n,1:8) = [0 0 2 0 0 0 0 0];   % k = 2, i = 1
    A(5*n+n,(n-1)*8+1:n*8) = pc(3,:);   % k = 2, i = n
    A(6*n+n,1:8) = [0 0 0 3*2 0 0 0 0];   % k = 3, i = 1
    A(7*n+n,(n-1)*8+1:n*8) = pc(4,:);   % k = 3, i = n
    %load A.mat    
    b_x = [waypoints(1,1:end-1)';waypoints(1,2:end)';zeros(6*n,1)];  % 8N*1
    b_y = [waypoints(2,1:end-1)';waypoints(2,2:end)';zeros(6*n,1)]; 
    b_z = [waypoints(3,1:end-1)';waypoints(3,2:end)';zeros(6*n,1)];
    coeff_x = reshape(A\b_x,8,n)';  % 8N*1
    coeff_y = reshape(A\b_y,8,n)'; 
    coeff_z = reshape(A\b_z,8,n)'; 
    coeff(:,1,:) = coeff_x;  
    coeff(:,2,:) = coeff_y;  
    coeff(:,3,:) = coeff_z;  
    
else             % only gives t and current state
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);  % find the first element > t

    if(t_index > 1)
        t = t - traj_time(t_index-1);  %  t - Si-1
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
    else
        % traj_tims is S, Ti = Si - Si-1  -> d0        
        scale = t/d0(t_index-1);
        desired_state.pos = squeeze(coeff(t_index-1,:,:))*[1 scale scale^2 scale^3 scale^4 scale^5 scale^6 scale^7]';
        desired_state.vel = squeeze(coeff(t_index-1,:,:))*[0 1 2*scale 3*scale^2 4*scale^3 5*scale^4 6*scale^5 7*scale^6]'/d0(t_index-1);
        desired_state.acc = squeeze(coeff(t_index-1,:,:))*[0 0 2*1*scale 3*2*scale^2 4*3*scale^3 5*4*scale^4 6*5*scale^5 7*6*scale^6]'/(d0(t_index-1)^2);
    end
    
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end

end

