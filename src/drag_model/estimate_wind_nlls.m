%options
interval_start=1;
interval_end=100;
%
chassis_states=logsout_quadcopter_package_delivery{4}.Values.Chassis;
%total mass
drone_and_pkg_mass=prod(pkgSize)*pkgDensity+drone_mass;
load_status=logsout_quadcopter_package_delivery{4}.Values.Load.status.Data;
mass_of_t=zeros(size(load_status)).';
mass_of_t(load_status>-0.5)=drone_and_pkg_mass;
mass_of_t(load_status<-0.5)=drone_mass;
%extract times
times=chassis_states.px.Time(interval_start:interval_end);
%extract observed accelerations
v_of_t=[chassis_states.vx;chassis_states.vy;chassis_states.vz];
obs_times=(times(interval_start:interval_end-1)+times(interval_start+1:interval_end))/2;
dvxdt=(v_of_t(1).Data(2:end)-v_of_t(1).Data(1:end-1))./(v_of_t(1).Time(2:end)-v_of_t(1).Time(1:end-1));
dvydt=(v_of_t(2).Data(2:end)-v_of_t(2).Data(1:end-1))./(v_of_t(2).Time(2:end)-v_of_t(2).Time(1:end-1));
dvzdt=(v_of_t(3).Data(2:end)-v_of_t(3).Data(1:end-1))./(v_of_t(3).Time(2:end)-v_of_t(3).Time(1:end-1));

a_obs=[dvxdt,dvydt,dvzdt];

%extract state history
position_history=[chassis_states.px.Data,chassis_states.py.Data,chassis_states.pz.Data].';
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

function handle=bind_vars(N,state_history,thrust_history,parameters,a_obs)
function diffs=accel_errors(flat_wind_estimate)
wind_estimate=reshape(flat_wind_estimate,3,N);
[predicted_acceleration,thrust_accel,drag_accel]=quadratic_drag_nominal_linear_acceleration(state_history,thrust_history,parameters,wind_estimate);
midpoint_predictions=(predicted_acceleration(:,1:end-1)+predicted_acceleration(:,2:end)).'/2;
difference=a_obs-midpoint_predictions;
diffs=difference(:);
end
handle=@accel_errors;
end
%estimate wind via NLLS
N=interval_end-interval_start+1;
guess_scale=1e-1;
wind_guess=guess_scale*randn(3,N);
estimated_wind_flat=lsqnonlin(bind_vars(N,state_history(:,interval_start:interval_end),thrust_history(:,interval_start:interval_end),parameters,a_obs(interval_start:interval_end-1,:)),wind_guess(:));
estimated_wind=reshape(estimated_wind_flat,3,N);
%plot
figure()
subplot(3,1,1)
hold on
plot(times,wind_guess(1,:),":o")
plot(times,wind_history(1,interval_start:interval_end))
plot(times,estimated_wind(1,:))
legend("Guess","Observed","Estimated")
ylabel("X Wind")
hold off

subplot(3,1,2)
hold on
plot(times,wind_guess(2,:),":o")
plot(times,wind_history(2,interval_start:interval_end))
plot(times,estimated_wind(2,:))
legend("Guess","Observed","Estimated")
ylabel("Y Wind")
hold off

subplot(3,1,3)
hold on
plot(times,wind_guess(3,:),":o")
plot(times,wind_history(3,interval_start:interval_end))
plot(times,estimated_wind(3,:))
legend("Guess","Observed","Estimated")
ylabel("Z Wind")
hold off
