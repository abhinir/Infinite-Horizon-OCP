function [A,B] = car_A_B(model, X_bar, U_bar)

X_next = forward_euler_car(0,X_bar,U_bar,model);

del_x_next_p = zeros(model.nx,100);
del_x_next_m = zeros(model.nx,100);
del_x_next = zeros(model.nx,100);
del_x = zeros(model.nx,100);
del_u = zeros(model.nu,100);
Y = [del_x;del_u];

for i = 1:1:100
    del_x(:,i) = normrnd(0,0.01,[model.nx,1]);
    del_u(:,i) = normrnd(0,0.01,[model.nu,1]);
    Y(:,i) = [del_x(:,i);del_u(:,i)];
    del_x_next_p(:,i) = ...
        forward_euler_car(0,X_bar+del_x(:,i), U_bar+del_u(:,i),model) - X_next;
    del_x_next_m(:,i) = ...
        forward_euler_car(0,X_bar-del_x(:,i), U_bar-del_u(:,i),model) - X_next;
    del_x_next(:,i) = (del_x_next_p(:,i) - del_x_next_m(:,i))./2;

end
P = del_x_next*(Y')*inv(Y*Y');
A = P(:,1:model.nx);
B = P(:,model.nx+1:end);
end
