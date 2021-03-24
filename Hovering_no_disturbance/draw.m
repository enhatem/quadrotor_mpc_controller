figure;

subplot(2,2,1)
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,2), 'r')
title('Body Position [m]');

subplot(2,2,2)
plot(out.STATES_SAMPLED(:,1), out.STATES_SAMPLED(:,3), 'r')
title('Vertical Velocity [m/s]');

subplot(2,2,3)
plot(out.CONTROLS(:,1), out.CONTROLS(:,2), 'r')
title('Thrust Force [N]');

subplot(2,2,4)
plot(disturbance(:,1), disturbance(:,2), 'r')
title('Road Excitation [m]');