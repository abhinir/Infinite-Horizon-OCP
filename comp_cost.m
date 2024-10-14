
model = model_register('car');
t_list = 20:10:200;
x_new = zeros(4,N+1);
x_new(:,1) = model.X0;
mpc_err = zeros(4,length(t_list));

for j = 1:1:length(t_list)
    N = t_list(j);
    cost_new = 0;
    for i=1:1:N
    
                
        % state_err = compute_state_error(x_new(:,i), x_nom(:,i), model.name);
    
        % u_new(:,i) = u_nom(:,i) - K(:,:,i)*state_err + ...
                                % alpha*kt(:,i);
        %fprintf("u_nom = %d; u_new = %d \n", u_nom(:,i), u_new(:,i));
        control1 = u1(i,1,1);
        control2 = u1(i,2,1);
        % if i<80
        %     control1 = u(i,1,i);
        %     control2 = u(i,2,i);
        % else
        %     control1 = u(i,1,i+2);
        %     control2 = u(i,2,i);
        % end
        u_nom = [control1;control2];
        state_err = compute_state_error(x_new(:,i), model.Xg, model.name);
    
        cost_new = cost_new + (0.5*state_err'*model.Q*state_err + ... 
                                0.5*u_nom'*model.R*u_nom);
    
        x_new(:,i+1) = model.state_prop(i, x_new(:,i), u_nom, model);
        
               
    end
    err = compute_state_error(x_new(:,N+1), model.Xg, model.name);
    term_cost_mpc80(j) = 0.5*err'*S*err;
    cost_new = cost_new + term_cost_mpc(j);
    cost_mpc_iter80(j) = cost_new;
    mpc_err(:,j) = err;
end

%%
% term_cost = zeros(1,10);
% for i = 2:1:11
%     term_cost(i-1) = 0.5*ilqr_final_state_error(:,i)'*S*ilqr_final_state_error(:,i);
% end
