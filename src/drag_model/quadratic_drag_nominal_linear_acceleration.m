function [linear_accel_in_world,thrust_accel, drag_accel]= quadratic_drag_nominal_linear_acceleration(state,control,parameters,wind_velocity)
%QUADRATIC_DRAG_NOMINAL_LINEAR_ACCELERATION compute linear acceleration
%assuming quadratic drag model
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
%       inertia xz, Cdxx, Cdyy, Cdzz]. I.e. we store upper triangular part
%       of inertia matrix and the diagonal of the quadratic drag
%       coefficient for force.
%   wind_velocity : 3 elements
%       global frame wind velocity vector
quat_world_to_body=state(5:8,:).';%maps vectors in body frame to vectors in world frame
v=state(9:11,:).';
mass=state(1,:).';
g=[0;0;-9.81];
Bf=zeros(3,4);
Bf(3,:)=1;

Cd=diag(parameters(8:10));

quat_body_to_world=quatinv(quat_world_to_body);
airspeed_in_body_frame=quatrotate(quat_world_to_body,v-wind_velocity.');
drag_force_in_world=quatrotate(quat_body_to_world,-(Cd*(vecnorm(airspeed_in_body_frame,2,1).'.*airspeed_in_body_frame.')).'/2);
thrust_accel=(quatrotate(quat_body_to_world,(Bf*control).')./mass).';
drag_accel=(drag_force_in_world./mass).';
linear_accel_in_world= g+thrust_accel+drag_accel;
end
