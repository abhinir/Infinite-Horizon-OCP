function Xf = forward_euler(t, X, U, model)
temp = X;
for i = 1:100
    temp = temp + cartpole_nl_ode(t,temp,U,model)*(model.dt/100);
end
Xf = temp;
end