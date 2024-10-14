function [state_dot] = toy_rao_nl_ode(t, state, U, model)
    
    state_dot = zeros(model.nx,1);
    
    state_dot = -state.^3 + U;
    
    
end