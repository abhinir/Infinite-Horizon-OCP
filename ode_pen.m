function out = ode_pen(t, state, U, model)
[time,y] = ode45(@(time,y)...
    pendulum_nl_ode(t,y,U,model),[t t+model.dt],state);
out = y(end,:)';

end