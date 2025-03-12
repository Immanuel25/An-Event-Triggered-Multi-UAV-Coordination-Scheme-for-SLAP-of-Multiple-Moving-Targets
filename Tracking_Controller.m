%%  Description: tracking S-T curve, a hybrid trajectory tracking and path following
%   Inputs:
%       t:                  current time
%       Ts:                 sampling time
%       p:                  vehicle position in 2D
%       psi:                vehicle heading
%       pd:                 desired S-T curve to be track   
%       pd_dot:             time derivative of pd
%       pd_gamma:           partial derivative of pd respect to
%                           the path parameter gamma
%       controller_type:    TypeI or TypeII
%       vd:                 desired speed profile for gamma_dot
%       Gains:              Controller tuning parameter
%       
%   Outputs:
%       u:                  vehicle linear and angular speed
%       gamma_dot_new:      desired derivative of gamma
%       e_pos:              position error in the Body frame - this is
%                           optional just for mornitoring
%       e_gamma:            the error on the derivative of gamma - this is
%                           optional, just for mornitoring
%% =========================================================================================================================
function [u, gamma_dot_new, e_pos, e_gamma] = Tracking_Controller(t,Ts,p1,p2,pd,pd_dot,pd_gamma,controller_type, gamma_dot_old, omegad, Gains, input, pd_ddot)
    persistent epsilon Delta_bar K kz gamma_ddot_max gamma_ddot_min;
    if (t == 0)
    % Set controller parameters for the first time
        epsilon = Gains.epsilon;
        Delta_bar = Gains.Delta_bar;   
        K = Gains.K;   
        kz = Gains.kz;         
        gamma_ddot_max = Gains.gamma_ddot_max;
        gamma_ddot_min = Gains.gamma_ddot_min;

    end
    R_IB = Rotation_matrix(p2(1),p2(2),p2(3));  % yaw, pitch, roll;
    e_pos = R_IB'*(p1-pd) - epsilon;    
    u = Delta_bar*(R_IB'*pd_dot - K*e_pos);  % compute linear and angular speeds
    if strcmp(controller_type, 'TypeI')
        gamma_dot_new = omegad;
        e_gamma = 0;
    elseif strcmp(controller_type, 'TypeII')
        % go with controller TypeII
        e_gamma = gamma_dot_old - omegad;
        gamma_ddot = -kz*e_gamma + e_pos'*R_IB'*pd_gamma;
        gamma_ddot = max(min(gamma_ddot,gamma_ddot_max),gamma_ddot_min);
        gamma_dot_new = gamma_dot_old + Ts*gamma_ddot;
    elseif strcmp(controller_type, 'TypeIII')        
        b=pd_ddot;                                               
        kappa=(pd_gamma(1)*b(2)-pd_gamma(2)*b(1))/norm(pd_gamma(1:2))^3;
        
        psiP = atan2(pd_dot(2,end),pd_dot(1,end));
        RI_F=[ cos(psiP)     sin(psiP);                      % From {I} to {F}
              -sin(psiP)     cos(psiP)]; 
        e_P=RI_F*(p1(1:2)-pd(1:2));
        s1=e_P(1);
        y1=e_P(2);
        hg = norm(pd_gamma);
        v_robot = input(1);
        r_robot = input(4);
        psi = p2(3,end);
        v_gamma = gamma_dot_old;
        
        psie=psi-psiP;
        
        psie = mod(psi - psiP + pi, 2*pi) - pi;
        [k1,k2,k3,k_delta,theta,Delta_h,vd]=ConPara1();

        delta=-theta*tanh(k_delta*y1);
        ydot=v_robot*sin(psie)-hg*kappa*v_gamma*s1; % kappa
        delta_dot=-theta*k_delta*(1-(tanh(k_delta*y1))^2)*ydot;
        psi_tilde=psie-delta;
        % Controller
        %u_d=.5;
        u_d=hg*vd-s1*0.05;
        uP=(u_d*cos(psie)+k3*s1);
        v_gamma=uP/hg;
        if psie==delta
           r=delta_dot-k5*y1*v_robot+kappa*v_gamma*hg;  %k5  
        else
           r=kappa*uP + delta_dot - k1*psi_tilde - k2*y1*u_d*(sin(psie) - sin(delta))/psi_tilde;
        end
        u=[u_d;0;0;r];
        
        gamma_dot_new = omegad;
        e_gamma = 0;        
    end
end
function [k1,k2,k3,k_delta,theta,Delta_h,vd]=ConPara1()
% Control parameters for Method 1 to Method 5
    k1=1;
    k2=0.01;
    k3=0.005;
    vd = 0.10;
    theta=pi/8;
    k_delta=0.5;
    Delta_h=3000;
end

% function [k1,k2,k3,k_delta,theta,Delta_h,vd]=ConPara1()
% % Control parameters for Method 1 to Method 5
%     k1=1;
%     k2=1;
%     k3=0.5;
%     vd = 0.13;
%     theta=pi/4;
%     k_delta=6;
%     Delta_h=100;
% end
