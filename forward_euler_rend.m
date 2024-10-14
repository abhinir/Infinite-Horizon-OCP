function Xf = forward_euler_rend(t, X, U, model)
temp = X;
for i = 1:1
    temp = temp + rendezvous_nl_ode(t,temp,U,model)*(model.dt/1);
end
Xf = temp;
end