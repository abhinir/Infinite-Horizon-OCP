function out = ode_att_con(t, state, U, model)
[time,y] = ode45(@(time,y)...
    att_con_nl_ode(t,y,U,model),[t t+model.dt],state);
out = y(end,:)';

end