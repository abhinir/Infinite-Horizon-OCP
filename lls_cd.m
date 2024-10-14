function [A,B] = lls_cd(x_bar, u_bar, dt)

x_next = x_bar + dt*(car_ode(x_bar,u_bar));

del_x_next_p = zeros(4,100);
del_x_next_m = zeros(4,100);
del_x_next = zeros(4,100);
del_x = zeros(4,100);
del_u = zeros(2,100);
Y = [del_x;del_u];

for i = 1:1:100
    del_x(:,i) = normrnd(0,0.01,[model.nx,1]);
    del_u(:,i) = normrnd(0,0.01,[model.nu,1]);
    Y(:,i) = [del_x(:,i);del_u(:,i)];
    del_x_next_p(:,i) = ...
        x_bar + dt*(car_ode(x_bar+del_x(:,i), u_bar+del_u(:,i))) - x_next;
    del_x_next_m(:,i) = ...
        x_bar + dt*(car_ode(x_bar-del_x(:,i), u_bar-del_u(:,i))) - x_next;
    del_x_next(:,i) = (del_x_next_p(:,i) - del_x_next_m(:,i))./2;

end
P = del_x_next*(Y')*inv(Y*Y');
A = P(:,1:model.nx);
B = P(:,model.nx+1:end);
end

