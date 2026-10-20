function [linear_accel_in_world,g,thrust_accel,drag_accel] = symbolic_quad_drag_lin_acc_test(state,control,parameters,wind_velocity,attitude)
% symbolic_quad_drag_lin_acc computes linear acceleration of drone
% assuming quadratic drag model and symbolic wind_velocity
%
%   state : 14 elements
%       [mass, x pos, y pos, z pos, quat scalar, quat i, quat j, quat k, x vel, y
%       vel, z vel, x ang vel, y ang vel, z ang vel]
%       position, velocity in world frame. Quaternion maps vectors in body 
%       frame to vectors in world frame. Ang vel expressed in body frame
%   control : 4 elements
%       force to produce at each rotor
%   parameters : 10 elements
%       [mass, inertia xx, inertia yy, inertia zz, inertia xy, inertia yz,
%       inertia xz, Cdxx, Cdyy, Cdzz]. i.e. we store upper triangular part
%       of inertia matrix and the diagonal of the quadratic drag
%       coefficient for force.
%   wind_velocity : 3 elements, symbolic
%       global frame wind velocity vector

% % Quaternion matrix should give rotation from reference frame to current frame.
% quat_world_to_body=state(5:8,:);% correct

v=state(9:11,:); % Extracting Chassis Velocity in x,y,z direction
mass=state(1,:); % Extracting Drone Mass
g=[0;0;-9.81]; % Acceleration due to gravity in world frame
air_rho = 1.225; % Density of air at sea level in kg/m^3

R_world_to_body = eul2rotm(attitude).'; % correct
R_body_to_world = transpose(R_world_to_body); % correct

% Extracting (Cd * Area) in each translational direction, from parameters matrix
Cd = R_world_to_body * diag(parameters(8:10));

airspeed_in_world_frame = -(v+wind_velocity); % Magnitude of airspeed in body frame

% Calculates the sum of magnitudes of thrust force from each propeller,then divides it by mass to calculate acceleration.
thrust_accel_in_body = [0;0;sum(control)] ./ mass; % Adds magnitudes acceleration produced by thrust from all 4 propellers.

airspeed_in_body_frame = R_world_to_body*airspeed_in_world_frame; % correct
magnitude_of_airspeed = sqrt(sum(airspeed_in_body_frame .* airspeed_in_body_frame)); % correct
drag_force_in_body = -1/2 * air_rho * magnitude_of_airspeed * Cd * airspeed_in_body_frame;
drag_force_in_world=R_body_to_world*drag_force_in_body; % correct
thrust_accel= R_body_to_world*thrust_accel_in_body;
drag_accel=drag_force_in_world./mass; % correct
linear_accel_in_world= g+thrust_accel+drag_accel;
end