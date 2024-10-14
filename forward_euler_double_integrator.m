function Xf = forward_euler_double_integrator(t, X, U, model)
temp = X;
for i = 1:1
    temp = temp + double_integrator_ode(t,temp,U,model)*(model.dt/1);
end
Xf = temp;
end