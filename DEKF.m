%%  This code implements the distributed EKF algorithm used in the paper

%   An Event-Triggered Multi-UAV Coordination Scheme for Simultaneous Tracking and 
%   Pursuit of Multiple Moving Targets

%   Authors     : Tua A. Tamba, Immanuel R. Santjoko, Yul Y. Nazaruddin, and Vebi Nadhira
%   Contact     : immanuel.raynaldo.s@gmail.com

%   More information : https://github.com/Immanuel25

%% =========== Event Triggered Distributed Extended Kalman Filter ==========

function [Tracker, Sims] = DEKF(Tracker,Ts,t,Sims,Gains)

% Tuning parameters
    W = Gains.W;
    V = Gains.V;
    Wt = Gains.Wt;
    T_meas = Sims.T_meas;
    
    if rem(t,T_meas)==0 
        %   there are measurement
        Sims.ktime(end+1) = t;
        alpha_values = [];
        for i = 1:Sims.N
            Tracker(i) =  local_correction(Tracker(i),V,Sims);
            
            Tracker(i).alpha(end+1,:) = zeros(1,Sims.M);
            for j = 1:Sims.M
                alpha_values(end+1) = trace(Wt * Tracker(i).P_hat(:,:,j) * Wt');
                Tracker(i).alpha(end,j) = alpha_values(end);
            end
        end
        
        Sims.event(end+1) = 1;
        for i = 1:Sims.N
            % Define the next tracker index cyclically (e.g., last tracker fuses with the first)
            nextIndex = mod(i, Sims.N) + 1;  
            % Fuse the current tracker with the next one
            Tracker(i) = fusion(Tracker(i), Tracker(nextIndex),Sims);
        end
    else
        %   there are no measurement
        for i = 1:Sims.N
            for j = 1:Sims.M
                Tracker(i).q_hat_cor(:,j) = Tracker(i).q_hat(:,end,j);      
                Tracker(i).Omega_cor(:,:,j) = inv(Tracker(i).P_hat(:,:,j));     
            end
        end
    end
    
    % Predicttion
    for i = 1:Sims.N
        Tracker(i) = prediction(Tracker(i), W, Ts,Sims);
        for j = 1:Sims.M
            if Tracker(i).q_hat(5,end,j)<0
                % estimated target speed must be +, if not, make it +
                Tracker(i).q_hat(5,end,j) = abs(Tracker(i).q_hat(5,end,j));
                Tracker(i).q_hat(4,end,j) = Tracker(i).q_hat(4,end,j)+pi;
            end
        end
    end
end
    
function Tracker = local_correction(Tracker,V,Sims)
    for j = 1:Sims.M
        q1_hat = Tracker.q_hat(1:3,end,j);   
        h_hat = norm(q1_hat-Tracker.p1(:,end));
        y = Tracker.range(j);
        Omega = inv(Tracker.P_hat(:,:,j));    
        z = Omega*Tracker.q_hat(:,end,j);  
        C = [(q1_hat - Tracker.p1(:,end))'/h_hat 0 0 0];
        y_err = y - h_hat + C*Tracker.q_hat(:,end,j); 
                    
        Tracker.z_tilde(:,j) = z + C'*V*y_err;                                      % will be transmited to neighbor, i.e. Tracker 3       = z_tilde;
        Tracker.Omega_tilde(:,:,j) = Omega + C'*V*C;    
    end
end
    
function Tracker = fusion(Tracker,neighbor,Sims)
    for j = 1:Sims.M
        Tracker_weight = neighbor.range(j)/(Tracker.range(j)+neighbor.range(j));
        neighbor_weight = 1-Tracker_weight;
        
        Tracker.z_cor(:,j) = Tracker_weight*Tracker.z_tilde(:,j) + neighbor_weight*neighbor.z_tilde(:,j);
        
        Tracker.Omega_cor(:,:,j) = Tracker_weight*Tracker.Omega_tilde(:,:,j) + neighbor_weight*neighbor.Omega_tilde(:,:,j);
        Tracker.q_hat_cor(:,j) = inv(Tracker.Omega_cor(:,:,j))*Tracker.z_cor(:,j);
    end
end
    
function Tracker = prediction(Tracker,W, Ts,Sims)
    Frb = [1 0 Ts; 0 1 0; 0 0 1];
    
    Tracker.q_hat(:,end+1,:) = zeros(6,1,Sims.M);
    Tracker.P_save(:,:,end+1,:) = zeros(6,6,1,Sims.M);
    for j = 1:Sims.M
        psi_hat_old = Tracker.psi_hat_old(j); 
        v_hat_old = Tracker.v_hat_old(j); 
        
        Frt = [-v_hat_old*sin(psi_hat_old) cos(psi_hat_old) 0; v_hat_old*cos(psi_hat_old) sin(psi_hat_old) 0; 0 0 0];
        F = [eye(3) Ts*Frt; zeros(3) Frb];
        
        q_hat_cor = Tracker.q_hat_cor(:,j);
        Omega_cor = Tracker.Omega_cor(:,:,j);
        
        
        
        q_hat_pre = F*q_hat_cor+[v_hat_old*sin(psi_hat_old)*(psi_hat_old)*Ts;-v_hat_old*cos(psi_hat_old)*(psi_hat_old)*Ts;0;0;0;0];
        Omega_pre = W - W*F/(Omega_cor + F'*W*F)*F'*W;
        
        Tracker.q_hat(:,end,j) = q_hat_pre;
        Tracker.P_hat(:,:,j) = inv(Omega_pre);
        Tracker.P_save(:,:,end,j) = Tracker.P_hat(:,:,j);
        
        Tracker.psi_hat_old(j) = Tracker.q_hat(4,end,j); 
        Tracker.v_hat_old(j) = Tracker.q_hat(5,end,j); 
    end
    % Centroid
    Tracker.q_hat_bar(:,end+1) = mean(Tracker.q_hat(:,end,:),3);
end