% This script sweeps various parameters in the Mathworks' Quadcopter Drone project.
% The original package delivery drone model is modified to make the drone hover at certain height.
% This script plots a consolidated graph of drone states and control inputs w.r.t wind speed, 
% for all the parameter sweep values.  

clc
clearvars

%% Set Parameter Sweep ranges

Total_Parameters = 6; % No.of parameters to be swept. (TODO)
len_parameter_sweep = 4; % Set the length of sweep values (TODO)

wind_speed_sweep = 1:4; % Define the range of wind speed (TODO)
len_wind_speed = length(wind_speed_sweep); % Size of wind speed sweep can be varied (TODO)

%define the parameters and their sweep values below: (TODO)
density_sweep = [1.25, 10, 20, 25]; % Define the range for material density
propeller_diameter_sweep = [0.254, 0.275, 0.3, 0.32]; % Define the range of propeller diameter
propeller_Kthrust_sweep = [0.1072, 0.2, 0.3, 0.35]; % Define the range for propeller thrust constant
motor_time_const_sweep = [0.0001, 0.01, 0.015, 0.02]; % Define the range for motor time constant
drag_Cd_Y_sweep = [0.001, 0.15, 0.4, 0.65]; % Define the range for aerodynamic drag in Y-direction
drag_pitch_sweep = [0.2, 50, 100, 150]; % Define the range for aerodynamic drag against pitch motion

% The below condition raises an error if the sweep length of all parameters is not same.
if (length(density_sweep) ~= length(propeller_diameter_sweep)) || (length(propeller_diameter_sweep) ~= length(propeller_Kthrust_sweep))...
        || (length(propeller_Kthrust_sweep) ~= length(motor_time_const_sweep)) || (length(motor_time_const_sweep) ~= length(drag_Cd_X_sweep))...
        || (length(drag_Cd_X_sweep) ~= length(drag_pitch_sweep)) || (length(drag_pitch_sweep) ~= len_parameter_sweep)

    error("The sweep length of all parameters must be same.");

end

% Below is the list of parameter names and their associated numbers:
% 1 -> Material Density
% 2 -> Propeller Diameter
% 3 -> Propeller Thrust Constant
% 4 -> Motor Time Constant
% 5 -> Drag in Y-direction
% 6 -> Drag in Pitch-direction

% Select the parameter name and number to be plotted
Para_Num = 6; % TODO
Para_Name = "Drag Coefficient in Y-direction"; % TODO
%% Creating placeholders and saving default values

openProject("/Users/kunaldeshmukh/Documents/MATLAB/Examples/R2026a/MathWorks-Teaching-Resources-Quadcopter-Modeling-Simulation-61167f7/QuadcopterDrone/Quadcopter_Drone.prj")

% Saving the default parameter values, to reset parameter after sweep completion.
default_rho_pla = rho_pla;
default_propeller_diameter = propeller.diameter;
default_propeller_KThrust = propeller.Kthrust;
default_motor_time_const = qc_motor.time_const;
default_Cd_Y = qd_drag.Cd_Y;
default_drag_Pitch = qd_drag.Pitch;


% Creating Placeholder matrices to store results recorded during parameter
% sweep. Parameter value changes along the row; while wind speed value changes
% along the column, and the parameters themselves change along the 3rd
% dimension of the matrix.


Res_Pitch = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_w1 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_w2 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_w3 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_w4 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_drag1 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_drag2 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_drag3 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_drag4 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_thrust1 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_thrust2 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_thrust3 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_thrust4 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_i1 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_i2 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_i3 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);
Res_i4 = zeros([len_parameter_sweep,len_wind_speed,Total_Parameters]);

%% Material Density Sweep

i = 1;
for rho_pla = density_sweep
    j = 1;
    for wind_speed = wind_speed_sweep

        sim("Models/quadcopter_package_delivery.slx");
        Res_Pitch(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Chassis.pitch.Data(1,1,end);
        Res_w1(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Prop1.w.Data(end,1);
        Res_w2(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Prop2.w.Data(end,1);
        Res_w3(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Prop3.w.Data(end,1);
        Res_w4(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Prop4.w.Data(end,1);
        Res_drag1(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Prop1.drag.Data(end,1);
        Res_drag2(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Prop2.drag.Data(end,1);
        Res_drag3(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Prop3.drag.Data(end,1);
        Res_drag4(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Prop4.drag.Data(end,1);
        Res_thrust1(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Prop1.thrust.Data(end,1);
        Res_thrust2(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Prop2.thrust.Data(end,1);
        Res_thrust3(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Prop3.thrust.Data(end,1);
        Res_thrust4(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Prop4.thrust.Data(end,1);
        Res_i1(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot1.i.Data(end,1);
        Res_i2(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot2.i.Data(end,1);
        Res_i3(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot3.i.Data(end,1);
        Res_i4(i,j,1) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot4.i.Data(end,1);

        j = j+1;
    end
    i = i+1;
end

rho_pla = default_rho_pla; % Resetting the default density value.

%% Propeller Diameter Sweep

i = 1;
for m = 1:length(propeller_diameter_sweep)
    propeller.diameter = propeller_diameter_sweep(m);
    j = 1;
    for wind_speed = wind_speed_sweep

        sim("Models/quadcopter_package_delivery.slx");
        Res_Pitch(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Chassis.pitch.Data(1,1,end);
        Res_w1(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Prop1.w.Data(end,1);
        Res_w2(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Prop2.w.Data(end,1);
        Res_w3(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Prop3.w.Data(end,1);
        Res_w4(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Prop4.w.Data(end,1);
        Res_drag1(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Prop1.drag.Data(end,1);
        Res_drag2(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Prop2.drag.Data(end,1);
        Res_drag3(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Prop3.drag.Data(end,1);
        Res_drag4(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Prop4.drag.Data(end,1);
        Res_thrust1(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Prop1.thrust.Data(end,1);
        Res_thrust2(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Prop2.thrust.Data(end,1);
        Res_thrust3(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Prop3.thrust.Data(end,1);
        Res_thrust4(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Prop4.thrust.Data(end,1);
        Res_i1(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot1.i.Data(end,1);
        Res_i2(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot2.i.Data(end,1);
        Res_i3(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot3.i.Data(end,1);
        Res_i4(i,j,2) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot4.i.Data(end,1);

        j = j+1;
    end
    i = i+1;
end

propeller.diameter = default_propeller_diameter; % Resetting the default propeller diameter value.

%% Propeller Thrust Constant (K_Thrust) Sweep

i = 1;
for m = 1:length(propeller_Kthrust_sweep)
    propeller.Kthrust = propeller_Kthrust_sweep(m);
    j = 1;
    for wind_speed = wind_speed_sweep

        sim("Models/quadcopter_package_delivery.slx");
        Res_Pitch(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Chassis.pitch.Data(1,1,end);
        Res_w1(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Prop1.w.Data(end,1);
        Res_w2(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Prop2.w.Data(end,1);
        Res_w3(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Prop3.w.Data(end,1);
        Res_w4(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Prop4.w.Data(end,1);
        Res_drag1(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Prop1.drag.Data(end,1);
        Res_drag2(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Prop2.drag.Data(end,1);
        Res_drag3(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Prop3.drag.Data(end,1);
        Res_drag4(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Prop4.drag.Data(end,1);
        Res_thrust1(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Prop1.thrust.Data(end,1);
        Res_thrust2(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Prop2.thrust.Data(end,1);
        Res_thrust3(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Prop3.thrust.Data(end,1);
        Res_thrust4(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Prop4.thrust.Data(end,1);
        Res_i1(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot1.i.Data(end,1);
        Res_i2(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot2.i.Data(end,1);
        Res_i3(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot3.i.Data(end,1);
        Res_i4(i,j,3) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot4.i.Data(end,1);

        j = j+1;
    end
    i = i+1;
end

propeller.Kthrust = default_propeller_KThrust; % Resetting the default Kthrust value.

%% Motor Time Constant Sweep

i = 1;
for m = 1:length(motor_time_const_sweep)
    qc_motor.time_const = motor_time_const_sweep(m);
    j = 1;
    for wind_speed = wind_speed_sweep

        sim("Models/quadcopter_package_delivery.slx");
        Res_Pitch(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Chassis.pitch.Data(1,1,end);
        Res_w1(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Prop1.w.Data(end,1);
        Res_w2(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Prop2.w.Data(end,1);
        Res_w3(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Prop3.w.Data(end,1);
        Res_w4(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Prop4.w.Data(end,1);
        Res_drag1(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Prop1.drag.Data(end,1);
        Res_drag2(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Prop2.drag.Data(end,1);
        Res_drag3(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Prop3.drag.Data(end,1);
        Res_drag4(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Prop4.drag.Data(end,1);
        Res_thrust1(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Prop1.thrust.Data(end,1);
        Res_thrust2(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Prop2.thrust.Data(end,1);
        Res_thrust3(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Prop3.thrust.Data(end,1);
        Res_thrust4(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Prop4.thrust.Data(end,1);
        Res_i1(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot1.i.Data(end,1);
        Res_i2(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot2.i.Data(end,1);
        Res_i3(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot3.i.Data(end,1);
        Res_i4(i,j,4) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot4.i.Data(end,1);

        j = j+1;
    end
    i = i+1;
end

qc_motor.time_const = default_motor_time_const; % Resetting the default motor time constant value.

%% Drag in Y-direction Sweep

i = 1;
for m = 1:length(drag_Cd_Y_sweep)
    qd_drag.Cd_Y = drag_Cd_Y_sweep(m); % Sets the current drag coefficient for the sweep
    j = 1;
    for wind_speed = wind_speed_sweep

        sim("Models/quadcopter_package_delivery.slx");
        Res_Pitch(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Chassis.pitch.Data(1,1,end);
        Res_w1(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Prop1.w.Data(end,1);
        Res_w2(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Prop2.w.Data(end,1);
        Res_w3(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Prop3.w.Data(end,1);
        Res_w4(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Prop4.w.Data(end,1);
        Res_drag1(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Prop1.drag.Data(end,1);
        Res_drag2(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Prop2.drag.Data(end,1);
        Res_drag3(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Prop3.drag.Data(end,1);
        Res_drag4(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Prop4.drag.Data(end,1);
        Res_thrust1(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Prop1.thrust.Data(end,1);
        Res_thrust2(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Prop2.thrust.Data(end,1);
        Res_thrust3(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Prop3.thrust.Data(end,1);
        Res_thrust4(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Prop4.thrust.Data(end,1);
        Res_i1(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot1.i.Data(end,1);
        Res_i2(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot2.i.Data(end,1);
        Res_i3(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot3.i.Data(end,1);
        Res_i4(i,j,5) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot4.i.Data(end,1);

        j = j+1;
    end
    i = i+1;
end

qd_drag.Cd_Y = default_Cd_Y; % Resetting the default density value.

%% Drag in Pitching direction Sweep

i = 1;
for m = 1:length(drag_pitch_sweep)
    qd_drag.Pitch = drag_pitch_sweep(m);
    j = 1;
    for wind_speed = wind_speed_sweep

        sim("Models/quadcopter_package_delivery.slx");
        Res_Pitch(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Chassis.pitch.Data(1,1,end);
        Res_w1(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Prop1.w.Data(end,1);
        Res_w2(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Prop2.w.Data(end,1);
        Res_w3(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Prop3.w.Data(end,1);
        Res_w4(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Prop4.w.Data(end,1);
        Res_drag1(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Prop1.drag.Data(end,1);
        Res_drag2(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Prop2.drag.Data(end,1);
        Res_drag3(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Prop3.drag.Data(end,1);
        Res_drag4(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Prop4.drag.Data(end,1);
        Res_thrust1(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Prop1.thrust.Data(end,1);
        Res_thrust2(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Prop2.thrust.Data(end,1);
        Res_thrust3(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Prop3.thrust.Data(end,1);
        Res_thrust4(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Prop4.thrust.Data(end,1);
        Res_i1(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot1.i.Data(end,1);
        Res_i2(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot2.i.Data(end,1);
        Res_i3(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot3.i.Data(end,1);
        Res_i4(i,j,6) = logsout_quadcopter_package_delivery{4}.Values.Motor.Mot4.i.Data(end,1);

        j = j+1;
    end
    i = i+1;
end

qd_drag.Pitch = default_drag_Pitch; % Resetting the default density value.

%% Plotting States/Inputs/Outputs for Parameter Sweep

% Plotting change in Drone pitch w.r.t wind speed, for different values of the swept parameter.
figure (1);
hold on;
title("Plot of Drone Pitch w.r.t Wind Speed, for various values of" + Para_Name)
plot(wind_speed_sweep,Res_Pitch(1,:,Para_Num))
plot(wind_speed_sweep,Res_Pitch(2,:,Para_Num))
plot(wind_speed_sweep,Res_Pitch(3,:,Para_Num))
plot(wind_speed_sweep,Res_Pitch(4,:,Para_Num))
xlabel("Wind Speed (meter/sec)")
ylabel("Pitch (radians)")
legend("Cd Pitch = 0.2","Cd Pitch = 50","Cd Pitch = 100","Cd Pitch = 150")
grid on;
hold off;

% Subtitle declared once, to be reused for multiple subplots below.
subtitle = ["Cd Pitch = 0.2","Cd Pitch = 50","Cd Pitch = 100","Cd Pitch = 150"];

% Plotting change in propeller 1 angular velocity w.r.t wind speed, for different values of the swept parameter.
figure (2);
hold on;
sgtitle("Plot of Propeller 1 Angular Velocity w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_w1(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("w1 (RPM)")
    grid on;
end
hold off;

% Plotting change in propeller 2 angular velocity w.r.t wind speed, for different values of the swept parameter.
figure (3);
hold on;
sgtitle("Plot of Propeller 2 Angular Velocity w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_w2(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("w2 (RPM)")
    grid on;
end
hold off;

% Plotting change in propeller 3 angular velocity w.r.t wind speed, for different values of the swept parameter.
figure (4);
hold on;
sgtitle("Plot of Propeller 3 Angular Velocity w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_w3(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("w3 (RPM)")
    grid on;
end
hold off;

% Plotting change in propeller 4 angular velocity w.r.t wind speed, for different values of the swept parameter.
figure (5);
hold on;
sgtitle("Plot of Propeller 4 Angular Velocity w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_w4(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("w4 (RPM)")
    grid on;
end
hold off;

% Plotting change in drag experienced by propeller 1 w.r.t wind speed, for different values of the swept parameter.
figure (6);
hold on;
sgtitle("Plot of Drag on Propeller 1 w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_drag1(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("drag1 (N)")
    grid on;
end
hold off;

% Plotting change in drag experienced by propeller 2 w.r.t wind speed, for different values of the swept parameter.
figure (7);
hold on;
sgtitle("Plot of Drag on Propeller 2 w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_drag2(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("drag2 (N)")
    grid on;
end
hold off;

% Plotting change in drag experienced by propeller 3 w.r.t wind speed, for different values of the swept parameter.
figure (8);
hold on;
sgtitle("Plot of Drag on Propeller 3 w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_drag3(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("drag3 (N)")
    grid on;
end
hold off;

% Plotting change in drag experienced by propeller 1 w.r.t wind speed, for different values of the swept parameter.
figure (9);
hold on;
sgtitle("Plot of Drag on Propeller 4 w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_drag4(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("drag4 (N)")
    grid on;
end
hold off;

% Plotting change in thrust produced by propeller 1 w.r.t wind speed, for different values of the swept parameter.
figure (10);
hold on;
sgtitle("Plot of Thrust produced by Propeller 1 w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_thrust1(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("thrust1 (N)")
    grid on;
end
hold off;

% Plotting change in thrust produced by propeller 2 w.r.t wind speed, for different values of the swept parameter.
figure (11);
hold on;
sgtitle("Plot of Thrust produced by Propeller 2 w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_thrust2(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("thrust2 (N)")
    grid on;
end
hold off;

% Plotting change in thrust produced by propeller 3 w.r.t wind speed, for different values of the swept parameter.
figure (12);
hold on;
sgtitle("Plot of Thrust produced by Propeller 3 w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_thrust3(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("thrust3 (N)")
    grid on;
end
hold off;

% Plotting change in thrust produced by propeller 4 w.r.t wind speed, for different values of the swept parameter.
figure (13);
hold on;
sgtitle("Plot of Thrust produced by Propeller 4 w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_thrust4(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("thrust4 (N)")
    grid on;
end
hold off;

% Plotting change in current required by Motor 1 w.r.t wind speed, for different values of the swept parameter.
figure (14);
hold on;
sgtitle("Plot of Current req. by Motor 1 w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_i1(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("i1 (A)")
    grid on;
end
hold off;

% Plotting change in current required by Motor 2 w.r.t wind speed, for different values of the swept parameter.
figure (15);
hold on;
sgtitle("Plot of Current req. by Motor 2 w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_i2(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("i2 (A)")
    grid on;
end
hold off;

% Plotting change in current required by Motor 3 w.r.t wind speed, for different values of the swept parameter.
figure (16);
hold on;
sgtitle("Plot of Current req. by Motor 3 w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_i3(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("i3 (A)")
    grid on;
end
hold off;

% Plotting change in current required by Motor 4 w.r.t wind speed, for different values of the swept parameter.
figure (17);
hold on;
sgtitle("Plot of Current req. by Motor 4 w.r.t Wind Speed, for various " + Para_Name)

for k = 1:4
    subplot(2,2,k);
    plot(wind_speed_sweep,Res_i4(k,:,Para_Num))
    title(subtitle(k))
    xlabel("Wind Speed (meter/sec)")
    ylabel("i4 (A)")
    grid on;
end
hold off;