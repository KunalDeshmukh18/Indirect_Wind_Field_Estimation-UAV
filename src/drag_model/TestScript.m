cd('/Users/kunaldeshmukh/Documents/MATLAB/Examples/R2026a/MathWorks-Teaching-Resources-Quadcopter-Modeling-Simulation-61167f7/Improved_NLLS_Estimation/');
openProject("Quadcopter_Drone.prj");
%% Running Simulation to obtain actual wind values, drone states
Initial_WindSpeed = 2;
Final_WindSpeed = 3;
Step_Time = 25; % Time at which wind speed changes value in a step manner.
Simulation_Time = 35; % Total Simulation Time
sim("quadcopter_package_delivery");
%% Initializing Parameters
interval_start = 2637; % Time index approx 5 sec before step time.
interval_end = 3770; % Time index approx 5 sec after step time.
drone_mass = 1.2726; % Mass of drone used for estimation.

% Calculating total mass (Drone Mass + Package Mass)
drone_and_pkg_mass=prod(pkgSize)*pkgDensity+drone_mass; % Multiplying package volume by density
load_status=(logsout_quadcopter_package_delivery{4}.Values.Load.status.Data); % Load Status (0 or 1) for entire simulation time period
Total_mass = drone_mass .* ones(size(load_status)).'; % Mass of the drone (+ package) for each timestep during simulation.

% Extracting Drone Chassis states data
chassis_states=logsout_quadcopter_package_delivery{4}.Values.Chassis;

% Extracting simulation time data
time = chassis_states.px.Time;

% Extracting observed drone chassis accelerations
Chassis_Velocity=[chassis_states.vx;chassis_states.vy;chassis_states.vz];

% Calculating the instantaneous acceleration of drone chassis
dvxdt=(Chassis_Velocity(1).Data(2:end)-Chassis_Velocity(1).Data(1:end-1)) ./ (Chassis_Velocity(1).Time(2:end)-Chassis_Velocity(1).Time(1:end-1));
dvydt=(Chassis_Velocity(2).Data(2:end)-Chassis_Velocity(2).Data(1:end-1)) ./ (Chassis_Velocity(2).Time(2:end)-Chassis_Velocity(2).Time(1:end-1));
dvzdt=(Chassis_Velocity(3).Data(2:end)-Chassis_Velocity(3).Data(1:end-1)) ./ (Chassis_Velocity(3).Time(2:end)-Chassis_Velocity(3).Time(1:end-1));

Chassis_Acceleration=[dvxdt,dvydt,dvzdt];

% Extracting Chassis state data:
position_history=[chassis_states.px.Data, chassis_states.py.Data, chassis_states.pz.Data].'; % Drone chassis position data
rpy_history=[squeeze(chassis_states.roll.Data), squeeze(chassis_states.pitch.Data), squeeze(chassis_states.yaw.Data)]; % Drone Chassis Roll-Pitch-Yaw data
quat_history=quaternion(rpy_history,"euler","XYZ","point"); % Converting Roll-Pitch-Yaw euler angles to quaternion form
velocity_history=[chassis_states.vx.Data,chassis_states.vy.Data,chassis_states.vz.Data].'; % Drone Chassis velocity data
angvel_history=[chassis_states.nPitch.Data,chassis_states.nRoll.Data,chassis_states.nYaw.Data].'; % Drone Chassis angular velocity data

state_history=[Total_mass;position_history;compact(quat_history).';velocity_history;angvel_history];

% Extracting thrust data of all propellers
thrust_history=[logsout_quadcopter_package_delivery{4}.Values.Prop1.thrust.Data,logsout_quadcopter_package_delivery{4}.Values.Prop2.thrust.Data,logsout_quadcopter_package_delivery{4}.Values.Prop3.thrust.Data,logsout_quadcopter_package_delivery{4}.Values.Prop4.thrust.Data].';

% Extracting wind velocity data throughout the simulation
wind_struct=logsout_quadcopter_package_delivery{1}.Values.wind;
wind_history=[wind_struct.vx.Data,wind_struct.vy.Data,wind_struct.vz.Data].';

% Constructing parameter vector
parameters=[drone_and_pkg_mass;zeros(6,1);qd_drag.Cd_X*qd_area.YZ;qd_drag.Cd_Y*qd_area.XZ;qd_drag.Cd_Z*qd_area.XY];

% Changing the current working directory to the directory containing this script.
cd("/Users/kunaldeshmukh/Documents/MATLAB/Examples/R2026a/MathWorks-Teaching-Resources-Quadcopter-Modeling-Simulation-61167f7/Improved_NLLS_Estimation/src/drag_model/");

N = interval_end-interval_start+1; % Total no.of intervals for which estimation will be performed.
predicted_acceleration = zeros(3,N); % Creates an null 3xN array.
actual_velocity = [(logsout_quadcopter_package_delivery{1}.Values.wind.vx.Data).';...
    (logsout_quadcopter_package_delivery{1}.Values.wind.vy.Data).';...
    (logsout_quadcopter_package_delivery{1}.Values.wind.vz.Data).'];

thrust_accel = zeros(3,N);
drag_accel = zeros(3,N);
grav = zeros(3,N);
attitude_history = [squeeze(chassis_states.yaw.Data), squeeze(chassis_states.pitch.Data), squeeze(chassis_states.roll.Data)]; % ZYX (Yaw, Pitch, Roll) Orientation
%% Test Function
for i=1:N
    % Selecting drone states data for each time interval
    these_w = actual_velocity(:,interval_start+i-1);
    X = state_history(:,interval_start+i-1);
    U = thrust_history(:,interval_start+i-1);
    attitude = attitude_history(interval_start+i-1,:);

    % Predicting Chassis acceleration
    [predicted_acceleration(:,i),grav(:,i),thrust_accel(:,i),drag_accel(:,i)] = symbolic_quad_drag_lin_acc_test(X,U,parameters,these_w,attitude);
end
%% Plotting the components of measured thrust acc(World Frame) under no wind speed condition (i.e no drag)
Initial_WindSpeed = 0;
Final_WindSpeed = 0;
Step_Time = 25; % Time at which wind speed changes value in a step manner.
Simulation_Time = 35; % Total Simulation Time
sim("quadcopter_package_delivery");

k = 2597; % Choose any time instant in the Drone hovering window
R_Inertia2Body = eul2rotm(attitude_history(k,:));
R_Body2Inertia = transpose(R_Inertia2Body);

acc_thrust_Body = [0;0;sum(thrust_history(:,k))] ./ drone_mass
acc_thrust_Inertia = R_Body2Inertia * [0;0;sum(thrust_history(:,k))] ./ drone_mass

%% Plotting the measured Thrust alongside the calculated Thrust
Total_Thrust_Acc_Measured_BodyFrame = (logsout_quadcopter_package_delivery{4}.Values.Prop1.thrust.Data(interval_start:interval_end) + ...
                        logsout_quadcopter_package_delivery{4}.Values.Prop2.thrust.Data(interval_start:interval_end) + ...
                        logsout_quadcopter_package_delivery{4}.Values.Prop3.thrust.Data(interval_start:interval_end) + ...
                        logsout_quadcopter_package_delivery{4}.Values.Prop4.thrust.Data(interval_start:interval_end)) ./ drone_mass;

% Calculating the magnitude of the predicted thrust acceleration, at each time step
pred_thrust_acc = zeros(3,length(thrust_accel));
pred_thrust_acc_mag = zeros(length(thrust_accel),1);

for k = 1:(interval_end - interval_start + 1)
    R_Inertia2Body = eul2rotm(attitude_history(interval_start + k -1,:));
    pred_thrust_acc(:,k) = R_Inertia2Body * thrust_accel(:,k);
    pred_thrust_acc_mag(k) = sqrt(pred_thrust_acc(1,k)^2 + pred_thrust_acc(2,k)^2 + pred_thrust_acc(3,k)^2); 
end

figure();
hold on;
plot(time(interval_start:interval_end), Total_Thrust_Acc_Measured_BodyFrame, Color = "blue")
plot(time(interval_start:interval_end), pred_thrust_acc_mag, Color = "red")
xlabel("Time (in sec)");
ylabel("acceleration (m/s^2)")
legend("Measured Thrust Acceleration", "Calculated Thrust Acceleration")
hold off;
%% Plotting Measured Drag alongside the Calculated Drag
figure();

subplot(3,1,1)
hold on;
plot(time(interval_start:interval_end), Measured_DragX.Data(interval_start:interval_end) ./ drone_mass, Color="blue")
plot(time(interval_start:interval_end), drag_accel(1,:), Color="red")
subtitle("Measured and Calculated Drag Acceleration of Drone along X-axis");
xlabel("Time (in sec)");
ylabel("acceleration (m/s^2)")
legend("Measured Drag Acceleration", "Calculated Drag Acceleration")
hold off;

subplot(3,1,2)
hold on;
plot(time(interval_start:interval_end), Measured_DragY.Data(interval_start:interval_end) ./ drone_mass, Color="blue")
plot(time(interval_start:interval_end), drag_accel(2,:), Color="red")
subtitle("Measured and Calculated Drag Acceleration of Drone along Y-axis");
xlabel("Time (in sec)");
ylabel("acceleration (m/s^2)")
legend("Measured Drag Acceleration", "Calculated Drag Acceleration")
hold off;

subplot(3,1,3)
hold on;
plot(time(interval_start:interval_end), Measured_DragZ.Data(interval_start:interval_end) ./ drone_mass, Color="blue")
plot(time(interval_start:interval_end), drag_accel(3,:), Color="red")
subtitle("Measured and Calculated Drag Acceleration of Drone along Z-axis");
xlabel("Time (in sec)");
ylabel("acceleration (m/s^2)")
legend("Measured Drag Acceleration", "Calculated Drag Acceleration")
hold off;
%% Plotting Predicted Acceleration and Calculated Acceleration

figure()

subplot(3,1,1)
hold on;
title("Predicted and Calculated Acceleration in X-direction");
plot(time(interval_start:interval_end), predicted_acceleration(1,:), Color="red")
plot(time(interval_start:interval_end), dvxdt(interval_start:interval_end), Color="blue")
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")
legend("Predicted Acceleration","Calculated Acceleration")
grid on;
hold off

subplot(3,1,2)
hold on;
title("Predicted and Calculated Acceleration in Y-direction");
plot(time(interval_start:interval_end), predicted_acceleration(2,:), Color="red")
plot(time(interval_start:interval_end), dvydt(interval_start:interval_end), Color="blue")
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")
legend("Predicted Acceleration","Calculated Acceleration")
grid on;
hold off

subplot(3,1,3)
hold on;
title("Predicted and Calculated Acceleration in Z-direction");
plot(time(interval_start:interval_end), predicted_acceleration(3,:), Color="red")
plot(time(interval_start:interval_end), dvzdt(interval_start:interval_end), Color="blue")
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")
legend("Predicted Acceleration","Calculated Acceleration")
grid on;
hold off
%% Plotting X,Y and Z components of acceleration due to thrust, drag and gravity

figure()

subplot(3,3,1)
hold on;
title("Thrust Acc X-Component");
plot(time(interval_start:interval_end), thrust_accel(1,:), Color="red")
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")
grid on;
hold off

subplot(3,3,2)
hold on;
title("Thrust Acc Y-Component");
plot(time(interval_start:interval_end), thrust_accel(2,:), Color="red")
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")
grid on;
hold off

subplot(3,3,3)
hold on;
title("Thrust Acc Z-Component");
plot(time(interval_start:interval_end), thrust_accel(3,:), Color="red")
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")
grid on;
hold off

subplot(3,3,4)
hold on;
title("Drag Acc X-Component");
plot(time(interval_start:interval_end), drag_accel(1,:), Color="blue")
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")
grid on;
hold off

subplot(3,3,5)
hold on;
title("Drag Acc Y-Component");
plot(time(interval_start:interval_end), drag_accel(2,:), Color="blue")
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")
grid on;
hold off

subplot(3,3,6)
hold on;
title("Drag Acc Z-Component");
plot(time(interval_start:interval_end), drag_accel(3,:), Color="blue")
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")
grid on;
hold off

subplot(3,3,7)
hold on;
title("Gravitational Acc X-Component");
plot(time(interval_start:interval_end), grav(1,:), Color="magenta")
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")
grid on;
hold off

subplot(3,3,8)
hold on;
title("Gravitational Acc Y-Component");
plot(time(interval_start:interval_end), grav(2,:), Color="magenta")
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")
grid on;
hold off

subplot(3,3,9)
hold on;
title("Gravitational Acc Z-Component");
plot(time(interval_start:interval_end), grav(3,:), Color="magenta")
xlabel("Time (sec)")
ylabel("Acceleration (m/s^2)")
grid on;
hold off