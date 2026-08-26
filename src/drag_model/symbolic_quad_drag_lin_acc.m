function [linear_accel_in_world] = symbolic_quad_drag_lin_acc(state,control,parameters,wind_velocity)
%symbolic_quad_drag_lin_acc compute linear acceleration
%assuming quadratic drag model and symbolic wind_velocity
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
%   wind_velocity : 3 elements, symbolic
%       global frame wind velocity vector
quat_world_to_body=state(5:8,:);%maps vectors in body frame to vectors in world frame
v=state(9:11,:);
mass=state(1,:);
g=[0;0;-9.81];
Bf=zeros(3,4);
Bf(3,:)=1;

Cd=diag(parameters(8:10));

R_world_to_body=quat2rotm(quat_world_to_body.').';
R_body_to_world=transpose(R_world_to_body);
airspeed_in_world_frame=v-wind_velocity;
thrust_accel_in_body=(Bf*control)./mass;
airspeed_in_body_frame=R_world_to_body*airspeed_in_world_frame;
magnitude_of_airspeed=sqrt(sum(airspeed_in_body_frame.*airspeed_in_body_frame));
drag_force_in_body=-magnitude_of_airspeed*Cd*airspeed_in_body_frame/2;
drag_force_in_world=R_body_to_world*drag_force_in_body;
thrust_accel=R_body_to_world*thrust_accel_in_body;
drag_accel=drag_force_in_world./mass;
linear_accel_in_world= g+thrust_accel+drag_accel;
end
