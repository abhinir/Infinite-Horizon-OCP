function [state_dot] = pendulum_nl_ode(t, state, U, model)
    
    state_dot = zeros(model.nx,1);
    
    state_dot(1) = state(2);
    state_dot(2) = - 3*model.g*sin(state(1))/model.L + 3*sin(U)/(model.m*(model.L^2));
    
    
end