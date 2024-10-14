function out = ode_rendezvous(t, state, U, model)
[time,y] = ode45(@(time,y)...
    rendezvous_nl_ode(t,y,U,model),[t t+model.dt],state);
out = y(end,:)';

end