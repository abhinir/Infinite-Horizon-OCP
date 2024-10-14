function out = ode_softLand(t, state, U, model)
[time,y] = ode45(@(time,y)...
    softLand_nl_ode(t,y,U,model),[t t+model.dt],state);
out = y(end,:)';

end