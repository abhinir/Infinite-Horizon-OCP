function out = rendezvous_nl_ode(t,x,u,model)

e1 = x(1);
e2 = x(2);
e3 = x(3);

ev1 = x(4);
ev2 = x(5);
ev3 = x(6); 

m = x(7);

r1 = [x(8);x(9);x(10)];
v1 = [x(11);x(12);x(13)];

r2 = r1 - [e1;e2;e3];

R1 = norm(r1);
R2 = norm(r2);

out = zeros(13,1);



out(1:3) = [ev1;ev2;ev3];
out(4:6) = -(model.mu/R1^3)* r1 + (model.mu/R2^3)* r2 - [u(1);u(2);u(3)]./m;
out(7) = -5e-4*sqrt(u'*u);
out(8:10) = v1;
out(11:13) = -(model.mu/R1^3)* r1;
end