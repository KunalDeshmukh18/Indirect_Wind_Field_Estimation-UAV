%options
plot_thrust=false;
plot_drag=false;
plot_pos_and_vel=false;
%
chassis_states=logsout_quadcopter_package_delivery{4}.Values.Chassis;
%total mass
drone_and_pkg_mass=prod(pkgSize)*pkgDensity+drone_mass;
load_status=logsout_quadcopter_package_delivery{4}.Values.Load.status.Data;
mass_of_t=zeros(size(load_status)).';
mass_of_t(load_status>-0.5)=drone_and_pkg_mass;
mass_of_t(load_status<-0.5)=drone_mass;
%extract times
times=chassis_states.px.Time;
%extract observed accelerations
v_of_t=[chassis_states.vx;chassis_states.vy;chassis_states.vz];
obs_times=(times(1:end-1)+times(2:end))/2;
dvxdt=(v_of_t(1).Data(2:end)-v_of_t(1).Data(1:end-1))./(v_of_t(1).Time(2:end)-v_of_t(1).Time(1:end-1));
dvydt=(v_of_t(2).Data(2:end)-v_of_t(2).Data(1:end-1))./(v_of_t(2).Time(2:end)-v_of_t(2).Time(1:end-1));
dvzdt=(v_of_t(3).Data(2:end)-v_of_t(3).Data(1:end-1))./(v_of_t(3).Time(2:end)-v_of_t(3).Time(1:end-1));

a_obs=[dvxdt,dvydt,dvzdt];

%extract state history
position_history=[chassis_states.px.Data,chassis_states.py.Data,chassis_states.pz.Data].';
rpy_history=[squeeze(chassis_states.roll.Data),squeeze(chassis_states.pitch.Data),squeeze(chassis_states.yaw.Data)].';
quat_history=quaternion(rpy_history.',"euler","XYZ","point");
velocity_history=[chassis_states.vx.Data,chassis_states.vy.Data,chassis_states.vz.Data].';
angvel_history=[chassis_states.nPitch.Data,chassis_states.nRoll.Data,chassis_states.nYaw.Data].';
state_history=[mass_of_t;position_history;compact(quat_history).';velocity_history;angvel_history];

%extract thrust history
thrust_history=[logsout_quadcopter_package_delivery{4}.Values.Prop1.thrust.Data,logsout_quadcopter_package_delivery{4}.Values.Prop2.thrust.Data,logsout_quadcopter_package_delivery{4}.Values.Prop3.thrust.Data,logsout_quadcopter_package_delivery{4}.Values.Prop4.thrust.Data].';

%extract winds
wind_struct=logsout_quadcopter_package_delivery{1}.Values.wind;
wind_history=[wind_struct.vx.Data,wind_struct.vy.Data,wind_struct.vz.Data].';

%build parameter vector
parameters=[drone_and_pkg_mass;zeros(6,1);qd_drag.Cd_X*qd_area.YZ;qd_drag.Cd_Y*qd_area.XZ;qd_drag.Cd_Z*qd_area.XY];

%predict acceleration
[predicted_acceleration,thrust_accel,drag_accel]=quadratic_drag_nominal_linear_acceleration(state_history,thrust_history,parameters,wind_history);

%plot
figure()
if plot_pos_and_vel
    plot_cols=3;
    subplot(3,3,1)
    plot(times,position_history(1,:))
    ylabel("X pos")
    subplot(3,3,2)
    plot(times,position_history(2,:))
    ylabel("Y pos")
    subplot(3,3,3)
    plot(times,position_history(3,:))
    ylabel("Z pos")
    
    subplot(3,3,4)
    plot(times,velocity_history(1,:))
    ylabel("X vel")
    subplot(3,3,5)
    plot(times,velocity_history(2,:))
    ylabel("Y vel")
    subplot(3,3,6)
    plot(times,velocity_history(3,:))
    ylabel("Z vel")

    plot_idx=6;
else
    plot_cols=1;
    plot_idx=0;
end
subplot(3,plot_cols,plot_idx+1)
hold on
plot(obs_times,a_obs(:,1))
plot(times,predicted_acceleration(1,:))
if plot_thrust
plot(times,thrust_accel(1,:))
end
if plot_drag
plot(times,drag_accel(1,:))
end
if plot_thrust && plot_drag
    legend("Observed","Predicted","Thrust","Drag")
elseif plot_thrust
    legend("Observed","Predicted","Thrust")
elseif plot_drag
    legend("Observed","Predicted","Drag")
else
    legend("Observed","Predicted")
end

ylabel("X accel")
hold off
subplot(3,plot_cols,plot_idx+2)
hold on
plot(obs_times,a_obs(:,2))
plot(times,predicted_acceleration(2,:))
if plot_thrust
plot(times,thrust_accel(2,:))
end
if plot_drag
plot(times,drag_accel(2,:))
end
if plot_thrust && plot_drag
    legend("Observed","Predicted","Thrust","Drag")
elseif plot_thrust
    legend("Observed","Predicted","Thrust")
elseif plot_drag
    legend("Observed","Predicted","Drag")
else
    legend("Observed","Predicted")
end
ylabel("Y accel")
hold off
subplot(3,plot_cols,plot_idx+3)
hold on
plot(obs_times,a_obs(:,3))
plot(times,predicted_acceleration(3,:))
if plot_thrust
plot(times,thrust_accel(3,:))
end
if plot_drag
plot(times,drag_accel(3,:))
end
if plot_thrust && plot_drag
    legend("Observed","Predicted","Thrust","Drag")
elseif plot_thrust
    legend("Observed","Predicted","Thrust")
elseif plot_drag
    legend("Observed","Predicted","Drag")
else
    legend("Observed","Predicted")
end
ylabel("Z accel")
hold off



