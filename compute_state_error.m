function [state_err] = compute_state_error(x, x_bar, modelName)

if strcmp(modelName, 'pendulum')

    state_err = (x - x_bar);
    %state_err(1) = atan2(sin(state_err(1)),cos(state_err(1)));

elseif strcmp(modelName, 'cartpole')
    state_err = (x - x_bar);
    %state_err(3) = atan2(sin(state_err(3)),cos(state_err(3)));

elseif strcmp(modelName, '1dcos')
    state_err = (x - x_bar);
elseif strcmp(modelName, 'softLand')
    state_err = [x(1:12) - x_bar(1:12);0];
elseif strcmp(modelName, 'rendezvous')
    state_err = x - x_bar;
elseif strcmp(modelName, 'att_con')
    state_err = (x - x_bar);
elseif strcmp(modelName, 'double_integrator')
    state_err = (x - x_bar);
elseif strcmp(modelName, 'toy_rao')
    state_err = (x - x_bar);
elseif strcmp(modelName, 'car')
    state_err = (x - x_bar);
end

end