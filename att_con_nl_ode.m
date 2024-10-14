function out = att_con_nl_ode(t,x,u,model)

psi = x(1);
theta = x(2);
phi = x(3);

w1 = x(4);
w2 = x(5);
w3 = x(6);


out = zeros(6,1);

% R = rotz(psi*180/pi)'*roty(theta*180/pi)'*rotx(phi*180/pi);
M = [0, sin(phi), cos(phi);...
     0, cos(theta)*cos(phi), -cos(theta)*sin(phi);...
     cos(theta), sin(theta)*sin(phi), sin(theta)*cos(phi)];

torque = [u(1);u(2);u(3)];
omega = [w1;w2;w3];

out(1:3) = (1/cos(theta))*M*omega;
out(4:6) = -inv(model.J)*cross(omega,model.J*omega) + inv(model.J)*torque;

end