close all;


figure
plot(0:0.01:3,lag_mul(1,:))
hold on
grid on
plot(0:0.001:3,vk(1,:))
xlabel('t')
ylabel('\lambda_{1}')

figure
plot(0:0.01:3,lag_mul(2,:))
hold on
grid on
plot(0:0.001:3,vk(2,:))
xlabel('t')
ylabel('\lambda_{2}')

figure
plot(0:0.01:3,lag_mul(3,:))
hold on
grid on
plot(0:0.001:3,vk(3,:))
xlabel('t')
ylabel('\lambda_{3}')

figure
plot(0:0.01:3,lag_mul(4,:))
hold on
grid on
plot(0:0.001:3,vk(4,:))
xlabel('t')
ylabel('\lambda_{4}')
