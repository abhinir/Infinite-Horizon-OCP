function Xf = forward_euler_toy_rao(t, X, U, model)
temp = X;
for i = 1:100
    temp = temp + toy_rao_nl_ode(t,temp,U,model)*(model.dt/100);
end
Xf = temp;
end