%cd('/Users/kunaldeshmukh/Documents/MATLAB/Examples/R2026a/MathWorks-Teaching-Resources-Quadcopter-Modeling-Simulation-61167f7/Dr_Gutow_Files/');
%openProject("Quadcopter_Drone.prj");
%% Setting Parameter Values and Running Simulation
Initial_WindSpeed = 1;
Final_WindSpeed = 2;
Step_Time = 40; % Time at which wind speed changes value in a step manner.
Simulation_Time = 80; % Total Simulation Time
sim("quadcopter_package_delivery");
%% NLLS Estimation
interval_start = 17900; % Time index approx 5 sec before step time.
interval_end = 18100; % Time index approx 5 sec after step time.
%total mass
drone_and_pkg_mass=prod(pkgSize)*pkgDensity+drone_mass;
load_status=logsout_quadcopter_package_delivery{4}.Values.Load.status.Data;
Total_mass = drone_mass .* ones(size(load_status)).';
chassis_states=logsout_quadcopter_package_delivery{4}.Values.Chassis;

%extract times
time = chassis_states.px.Time;

%extract observed accelerations
Chassis_Velocity=[chassis_states.vx;chassis_states.vy;chassis_states.vz];

% Calculating the instantaneous acceleration of drone chassis
dvxdt=(Chassis_Velocity(1).Data(2:end)-Chassis_Velocity(1).Data(1:end-1)) ./ (Chassis_Velocity(1).Time(2:end)-Chassis_Velocity(1).Time(1:end-1));
dvydt=(Chassis_Velocity(2).Data(2:end)-Chassis_Velocity(2).Data(1:end-1)) ./ (Chassis_Velocity(2).Time(2:end)-Chassis_Velocity(2).Time(1:end-1));
dvzdt=(Chassis_Velocity(3).Data(2:end)-Chassis_Velocity(3).Data(1:end-1)) ./ (Chassis_Velocity(3).Time(2:end)-Chassis_Velocity(3).Time(1:end-1));

Chassis_Acceleration=[dvxdt,dvydt,dvzdt];

%extract state history
position_history=[chassis_states.px.Data, chassis_states.py.Data, chassis_states.pz.Data].';
rpy_history=[squeeze(chassis_states.roll.Data), squeeze(chassis_states.pitch.Data), squeeze(chassis_states.yaw.Data)];
quat_history=quaternion(rpy_history,"euler","XYZ","point");
velocity_history=[chassis_states.vx.Data,chassis_states.vy.Data,chassis_states.vz.Data].';
angvel_history=[chassis_states.nPitch.Data,chassis_states.nRoll.Data,chassis_states.nYaw.Data].';

state_history=[Total_mass;position_history;compact(quat_history).';velocity_history;angvel_history];

%extract thrust history
thrust_history=[logsout_quadcopter_package_delivery{4}.Values.Prop1.thrust.Data,logsout_quadcopter_package_delivery{4}.Values.Prop2.thrust.Data,logsout_quadcopter_package_delivery{4}.Values.Prop3.thrust.Data,logsout_quadcopter_package_delivery{4}.Values.Prop4.thrust.Data].';

%extract winds
wind_struct=logsout_quadcopter_package_delivery{1}.Values.wind;
wind_history=[wind_struct.vx.Data,wind_struct.vy.Data,wind_struct.vz.Data].';
%build parameter vector
parameters=[drone_and_pkg_mass;zeros(6,1);qd_drag.Cd_X*qd_area.YZ;qd_drag.Cd_Y*qd_area.XZ;qd_drag.Cd_Z*qd_area.XY];

%cd("/Users/kunaldeshmukh/Documents/MATLAB/Examples/R2026a/MathWorks-Teaching-Resources-Quadcopter-Modeling-Simulation-61167f7/Dr_Gutow_Files/src/drag_model/");

%% estimate wind via NLLS
N=interval_end-interval_start+1;
guess_scale=1e-1; % Scaling factor
wind_guess=guess_scale*randn(3,N);%+wind_history(:,interval_start:interval_end);
wind_change_cost=1e-4*eye(3);
regularization=1e-5;
w_est_vars=optimvar("w",3*N);
wind_vector_syms=reshape(w_est_vars,3,N);
function [lsqobj, predicted_acceleration]=symbolic_loss(w,wind_change_cost,regularization,state_history,thrust_history,a_obs,parameters,interval_start,interval_end)
    N=interval_end-interval_start+1;
    predicted_acceleration=optimexpr(3,N);
    wind_vector_syms=reshape(w,3,N);
    for i=1:N
        these_w=wind_vector_syms(:,i);
        X=state_history(:,interval_start+i-1);
        U=thrust_history(:,interval_start+i-1);
        predicted_acceleration(:,i)=symbolic_quad_drag_lin_acc(X,U,parameters,these_w);
    end
    midpoint_predictions=(predicted_acceleration(:,1:end-1)+predicted_acceleration(:,2:end)).'/2;
    difference=a_obs(interval_start:interval_end-1,:)-midpoint_predictions;
    diffs=difference(:);
    wind_diffs=wind_vector_syms(:,2:end)-wind_vector_syms(:,1:end-1);
    wind_diff_cost_contrib=sum((sqrt(wind_change_cost)*wind_diffs).^2,["all"]);
    lsqobj=sum(diffs.^2)+wind_diff_cost_contrib+regularization*sum(wind_vector_syms.^2,["all"]);%penalize acceleration error, change in wind, and any wind being too nonzero
end
fprintf("Building loss expression: ")
tic
[lsqobj, predicted_acceleration] =symbolic_loss(w_est_vars,wind_change_cost,regularization,state_history,thrust_history,Chassis_Acceleration,parameters,interval_start,interval_end);
toc
fprintf("Building Problem object: ")
tic
problem=optimproblem(Objective=lsqobj);
toc
%% Solve
x0.w=wind_guess(:);
options=optimoptions(@lsqnonlin,Display="iter");
tic
[sol,loss,exitFlag]=solve(problem,x0,Options=options);
toc
estimated_wind=reshape(sol.w,3,N);
converged_accel=evaluate(predicted_acceleration,sol);
%% plot
interval_times=time(interval_start:interval_end);

figure()
subplot(3,2,1)
hold on
plot(interval_times,wind_guess(1,:),":oy")
plot(interval_times,wind_history(1,interval_start:interval_end),"b")
plot(interval_times,estimated_wind(1,:),"m")
legend("Guess","Observed","Estimated")
ylabel("X Wind")
hold off

subplot(3,2,3)
hold on
plot(interval_times,wind_guess(2,:),":oy")
plot(interval_times,wind_history(2,interval_start:interval_end),"b")
plot(interval_times,estimated_wind(2,:),"m")
legend("Guess","Observed","Estimated")
ylabel("Y Wind")
hold off

subplot(3,2,5)
hold on
plot(interval_times,wind_guess(3,:),":oy")
plot(interval_times,wind_history(3,interval_start:interval_end),"b")
plot(interval_times,estimated_wind(3,:),"m")
legend("Guess","Observed","Estimated")
ylabel("Z Wind")
hold off


subplot(3,2,2)
hold on
plot(interval_times,Chassis_Acceleration(interval_start:interval_end,1),"b")
plot(interval_times,converged_accel(1,:),"m")
legend("Observed","Converged")
ylabel("X Acc")
hold off

subplot(3,2,4)
hold on
plot(interval_times,Chassis_Acceleration(interval_start:interval_end,2),"b")
plot(interval_times,converged_accel(2,:),"m")
legend("Observed","Converged")
ylabel("Y Acc")
hold off

subplot(3,2,6)
hold on
plot(interval_times,Chassis_Acceleration(interval_start:interval_end,3),"b")
plot(interval_times,converged_accel(3,:),"m")
legend("Observed","Converged")
ylabel("Z Acc")
hold off
