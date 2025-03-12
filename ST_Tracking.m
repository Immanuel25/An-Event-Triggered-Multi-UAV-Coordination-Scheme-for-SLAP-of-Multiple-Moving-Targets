function Tracker = ST_Tracking(Tracker, Targets, t, Ts, TTC_type, Gains, ts)
    gamma = Tracker.gamma;
    gamma_dot = Tracker.gamma_dot;
    p1 = Tracker.p1;
    p2 = Tracker.p2;
    pd = Tracker.pd;
    pd_dot = Tracker.pd_dot;
    pd_ddot = Tracker.pd_ddot;
    q_hat = Tracker.q_hat_bar(:,end);
    q1 = Targets.q1_bar(:,end);
    qv(1) = Targets.q2_bar(1,end)*cos(Targets.q2_bar(2,end));
    qv(2) = Targets.q2_bar(1,end)*sin(Targets.q2_bar(2,end));
    qv(3) = 0;

    omega = Tracker.omega;
    r = Gains.r;
    phi = Tracker.phi;

    roll  =  Tracker.p2(1,end);
    pitch =  Tracker.p2(2,end);
    yaw  =  Tracker.p2(3,end);

    E_Pursuit = Tracker.E_Pursuit;

    % Update desired S-T curve
    r_gamma = [r*cos(gamma(end) + phi);                                      % spatial path (circle about target)
               r*sin(gamma(end) + phi);
               20]; 
    dr_gamma = [-r*sin(gamma(end) + phi);                                    % partial derivative of r respect to gamma
                 r*cos(gamma(end) + phi);
                 0];  
    pd(:,end+1) = q1(:,end) + r_gamma;                                       % update S-T curve
    pd_dot(:,end+1) = qv(:,end) + dr_gamma*omega(end);                            % update derivative of S-T curve
%     pd_dot = diff(pd)/ts;
    pd_ddot = diff(pd_dot)/ts;
                             
% Call ST tracking controller
    q1_hat = q_hat(1:3,end);                                                 % estimate of target trajectory                                              
    psi_hat = q_hat(4,end);
    v_hat = q_hat(5,end);
    qv_hat = [v_hat*cos(psi_hat); v_hat*sin(psi_hat); 0];                                                 % estimate of target velocity
    
    pd_hat = q1_hat  + r_gamma ;
    pd_dot_hat = qv_hat + dr_gamma*omega(end);                                    % time derivative of pd                          
    pd_gamma = dr_gamma;                                                    % partial derivative of pd respect to gamma
    
    % Call ST tracking controller
    [Tracker.Input(:,end+1), gamma_dot(end+1), e_pos_hat, e_gamma] ... 
     = Tracking_Controller(t,Ts,p1(:,end),p2(:,end),pd_hat,pd_dot_hat,pd_gamma, TTC_type, gamma_dot(end),omega(end), Gains, Tracker.Input(:,end),pd_ddot(:,end));
 
    Tracker.Input(:,end) = max(min(Tracker.Input(:,end),Tracker.InputMax),Tracker.InputMin);          % saturate the linear speed and angular speed    
    
% Compute position error w.r.t. q - true target trajectory
    R_IB =  Rotation_matrix(roll,pitch,yaw);       

    e_pos_q = R_IB'*(p1(:,end) - pd(:,end)) - Gains.epsilon; 

    E_Pursuit(end+1) = norm([e_pos_q; e_gamma]);                           % update pursuit error
% ========================================================
    Tracker.pd = pd;
    Tracker.pd_dot = pd_dot;
    Tracker.pd_ddot = pd_ddot;
    Tracker.gamma = gamma;
    Tracker.gamma_dot = gamma_dot;
    Tracker.E_Pursuit = E_Pursuit;
end    




