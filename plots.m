clear all
close all
d = testDataDecoder("statedata_success3.txt")

%%
close all
figure
plot(d.time,d.velocity_x,d.time,d.velocity_y,d.time,d.velocity_z)
xlabel('Time (s)')
ylabel('Velocity (cm/s)')
legend('x','y','z')
title('Velocities')

figure
plot(d.time,d.pitch,d.time,d.roll,d.time,d.yaw)
xlabel('Time (s)')
ylabel('Angle (degrees)')
legend('Pitch','Roll','Yaw')
ylim([-180 180])
title('Angles')

figure
plot(d.time,d.tvec_x)
hold on
plot(d.time,d.tvec_y)
plot(d.time,d.tvec_z)
hold off
legend('tvec x','tvec y','tvec z')
title('Translation vectors')

figure
subplot(2,1,1)
plot(d.time,d.tvec_x,d.time,d.ref_x)
title('X translation vector')
legend('X position','X reference')
subplot(2,1,2)
plot(d.time,-d.control_leftRight)
title('Control LR')

figure
subplot(2,1,1)
plot(d.time,d.tvec_y,d.time,d.ref_y)
title('Y translation vector')
legend('Y position','Y reference')
subplot(2,1,2)
plot(d.time,d.control_upDown)
title('Control UD')

figure
subplot(2,1,1)
plot(d.time,d.tvec_z,d.time,d.ref_z)
title('Z translation vector')
legend('Z position','Z reference')
subplot(2,1,2)
plot(d.time,-d.control_forwardBack)
title('Control FB')

figure
plot(d.time,d.rvec_y)
hold on
plot(d.time,d.rvec_z)
hold off
legend('rvec y','rvec z')
title('Rotation vectors')

figure
plot(d.time,d.rvec_x)
legend('rvec x')
title('Rotation vectors')