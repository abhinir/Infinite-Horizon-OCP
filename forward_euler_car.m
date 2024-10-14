function Xf = forward_euler_car(t, X, U, model)
temp = X;
for i = 1:100
    temp = temp + car_nl_ode(t,temp,U,model)*(model.dt/100);
end
Xf = temp;
end