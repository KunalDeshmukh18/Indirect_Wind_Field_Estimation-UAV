import matlab.engine
import numpy as np
import matplotlib.pyplot as plt
#import os

# Set current working directory to the 'Quadcopter_Drone.prj' location:-
# '/Users/kunaldeshmukh/Documents/MATLAB/Examples/R2026a/MathWorks-Teaching-Resources-Quadcopter-Modeling-Simulation-61167f7/QuadcopterDrone'

eng = matlab.engine.start_matlab()
eng.eval("openProject('Quadcopter_Drone.prj')", nargout = 1)

# Set parameter values
eng.workspace['wind_speed'] = 0
#eng.workspace['propeller.diameter'] = 0.254 # Change the parameter values and names as required.

sweep_value = np.arange(0,5,1) # Creates an array of wind_speed values (from 0-4 m/s, with gap of 1 m/s)

# Creating placeholder arrays for different variables
px, py, pz = np.zeros(len(sweep_value)), np.zeros(len(sweep_value)), np.zeros(len(sweep_value))
pitch, roll, yaw = np.zeros(len(sweep_value)), np.zeros(len(sweep_value)), np.zeros(len(sweep_value))
w1, w2, w3, w4 = np.zeros(len(sweep_value)), np.zeros(len(sweep_value)), np.zeros(len(sweep_value)), np.zeros(len(sweep_value))
drag1, drag2, drag3, drag4 = np.zeros(len(sweep_value)), np.zeros(len(sweep_value)), np.zeros(len(sweep_value)), np.zeros(len(sweep_value))
thrust1, thrust2, thrust3, thrust4 = np.zeros(len(sweep_value)), np.zeros(len(sweep_value)), np.zeros(len(sweep_value)), np.zeros(len(sweep_value))
i1, i2, i3, i4 = np.zeros(len(sweep_value)), np.zeros(len(sweep_value)), np.zeros(len(sweep_value)), np.zeros(len(sweep_value))
Battery_SOC, Battery_Temp = np.zeros(len(sweep_value)), np.zeros(len(sweep_value))

i = 0
for speed in sweep_value:
    eng.workspace['wind_speed'] = speed
    eng.eval("simOut = sim('quadcopter_package_delivery')", nargout = 0)

    px[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Chassis.px.Data(end,1)")
    py[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Chassis.py.Data(end,1)")
    pz[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Chassis.pz.Data(end,1)")

    roll[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Chassis.roll.Data(1,1,end)")
    pitch[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Chassis.pitch.Data(1,1,end)")
    yaw[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Chassis.yaw.Data(1,1,end)")

    w1[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Prop1.w.Data(end,1)")
    w2[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Prop2.w.Data(end,1)")
    w3[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Prop3.w.Data(end,1)")
    w4[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Prop4.w.Data(end,1)")

    drag1[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Prop1.drag.Data(end,1)")
    drag2[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Prop2.drag.Data(end,1)")
    drag3[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Prop3.drag.Data(end,1)")
    drag4[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Prop4.drag.Data(end,1)")

    thrust1[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Prop1.thrust.Data(end,1)")
    thrust2[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Prop2.thrust.Data(end,1)")
    thrust3[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Prop3.thrust.Data(end,1)")
    thrust4[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Prop4.thrust.Data(end,1)")

    i1[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Motor.Mot1.i.Data(end,1)")
    i2[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Motor.Mot2.i.Data(end,1)")
    i3[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Motor.Mot3.i.Data(end,1)")
    i4[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Motor.Mot4.i.Data(end,1)")

    #Battery_SOC[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Motor.Battery.SOC.Data(end,1)")
    #Battery_Temp[i] = eng.eval("logsout_quadcopter_package_delivery{4}.Values.Motor.Battery.degC.Data(end,1)")

    i += 1

figure,axes = plt.subplots(nrows = 2, ncols = 4, layout = 'constrained')

figure.suptitle('Motor Time Constant = 0.02')

axes[0,0].plot(sweep_value, px, label = 'px')
axes[0,0].plot(sweep_value, py, label = 'py')
axes[0,0].plot(sweep_value, pz, label = 'pz')
axes[0,0].set_xlabel('Wind Speed (m/s)')
axes[0,0].set_ylabel('Position (m)')
axes[0,0].set_title('Measured Position')
axes[0,0].legend()
axes[0,0].grid(True)

axes[0,1].plot(sweep_value, roll, label = 'roll')
axes[0,1].plot(sweep_value, pitch, label = 'pitch')
axes[0,1].plot(sweep_value, yaw, label = 'yaw')
axes[0,1].set_xlabel('Wind Speed (m/s)')
axes[0,1].set_ylabel('Angle (rad)')
axes[0,1].set_title('Measured Pose')
axes[0,1].legend()
axes[0,1].grid(True)

axes[0,2].plot(sweep_value, w1, label = 'w1')
axes[0,2].plot(sweep_value, w4, label = 'w4')
axes[0,2].set_xlabel('Wind Speed (m/s)')
axes[0,2].set_ylabel('angular velocity (RPM)')
axes[0,2].set_title('Angular Velocity')
axes[0,2].legend()
axes[0,2].grid(True)

axes[0,3].plot(sweep_value, w2, label = 'w2')
axes[0,3].plot(sweep_value, w3, label = 'w3')
axes[0,3].set_xlabel('Wind Speed (m/s)')
axes[0,3].set_ylabel('angular velocity (RPM)')
axes[0,3].set_title('Angular Velocity')
axes[0,3].legend()
axes[0,3].grid(True)

axes[1,0].plot(sweep_value, drag1, label = 'Prop1')
axes[1,0].plot(sweep_value, drag4, label = 'Prop4')
axes[1,0].set_xlabel('Wind Speed (m/s)')
axes[1,0].set_ylabel('drag (N)')
axes[1,0].set_title('Propeller Drag')
axes[1,0].legend()
axes[1,0].grid(True)

axes[1,1].plot(sweep_value, drag2, label = 'Prop2')
axes[1,1].plot(sweep_value, drag3, label = 'Prop3')
axes[1,1].set_xlabel('Wind Speed (m/s)')
axes[1,1].set_ylabel('Drag (N)')
axes[1,1].set_title('Propeller Drag')
axes[1,1].legend()
axes[1,1].grid(True)

axes[1,2].plot(sweep_value, thrust1, label = 'Prop1')
axes[1,2].plot(sweep_value, thrust2, label = 'Prop2')
axes[1,2].plot(sweep_value, thrust3, label = 'Prop3')
axes[1,2].plot(sweep_value, thrust4, label = 'Prop4')
axes[1,2].set_xlabel('Wind Speed (m/s)')
axes[1,2].set_ylabel('Thrust (N)')
axes[1,2].set_title('Propeller Thrust')
axes[1,2].legend()
axes[1,2].grid(True)

axes[1,3].plot(sweep_value, i1, label = 'Mot1')
axes[1,3].plot(sweep_value, i2, label = 'Mot2')
axes[1,3].plot(sweep_value, i3, label = 'Mot3')
axes[1,3].plot(sweep_value, i4, label = 'Mot4')
axes[1,3].set_xlabel('Wind Speed (m/s)')
axes[1,3].set_ylabel('Current (A)')
axes[1,3].set_title('Motor Current')
axes[1,3].legend()
axes[1,3].grid(True)

plt.show()

eng.quit()