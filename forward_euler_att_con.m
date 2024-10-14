function Xf = forward_euler_att_con(t, X, U, model)
temp = X;
for i = 1:1
    temp = temp + att_con_nl_ode(t,temp,U,model)*(model.dt/1);
end
Xf = temp;
end