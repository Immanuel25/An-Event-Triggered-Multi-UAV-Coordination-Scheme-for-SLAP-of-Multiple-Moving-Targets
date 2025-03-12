%%  This code implemenSims.Ts the distributed EKF algorithm used in the paper

%   An Event-Triggered Multi-UAV Coordination Scheme for Simultaneous Tracking and 
%   Pursuit of Multiple Moving TargeSims.Ts

%   Authors     : Tua A. Tamba, Immanuel R. Santjoko, Yul Y. Nazaruddin, and Vebi Nadhira
%   Contact     : immanuel.raynaldo.s@gmail.com

%   More information : https://github.com/Immanuel25

%% ========================== Simultaneous Tracking and Purusit ==================================

function SLAP(Sims)
    
% ----------------------------------- Tracker Model Parameters ------------------------------------------------------

    TrackerData = [ ...
        55, -60,  25,    pi,     0,     0,  5.4978,             0; 
        80,  35, -50, -pi/4,     0,  pi/2,  4.7124,   2*pi/Sims.N;
       -70,  75,  -4,     0,     0,     0,  4.0000,   4*pi/Sims.N;
        40, -25,  20,     0,  pi/2,     0,  5.4978,   6*pi/Sims.N;
       -25,  65, -30,     0,     0, -pi/2,  4.7124,   8*pi/Sims.N;
        35, -15,  45, -pi/2,     0,     0,  4.0000,  10*pi/Sims.N;
       -15,  35,  15,  pi/2,     0, -pi/2,  5.4978,  12*pi/Sims.N;
        35, -25,  10,  pi/4,     0, -pi/4,  4.7124,  14*pi/Sims.N;
        15, -35,   5,     0, -pi/4,     0,  4.0000,  16*pi/Sims.N ...
    ];


    for i = 1:Sims.N
        Tracker(i).p1 = TrackerData(i,1:3)';  % Position
        if strcmp(Sims.TTC_type, 'TypeIII'), Tracker(i).p1(3)=20; end
        Tracker(i).p2 = TrackerData(i,4:6)';  % Orientation
        if strcmp(Sims.TTC_type, 'TypeIII'), Tracker(i).p2(1:2)=[0;0]; end
        Tracker(i).gamma = TrackerData(i,7);
        Tracker(i).phi = TrackerData(i,8);         
        Tracker(i).p = [Tracker(i).p1; Tracker(i).p2];  
        Tracker(i).Input = [0;0;0;0];                          
        Tracker(i).InputMin = [0; -1; -1; -1];          
        Tracker(i).InputMax = [10; 1; 1; 0.4];       
        Tracker(i).gamma_dot = 0; 
    end
    if Sims.N == 2
        Tracker(2).phi = pi/2;
    end

    % ----------------------------------- Cooperative Localization Parameter------------------------------------------------------

    for i = 1:Sims.N   
        q1_hat_values = [ ...
             5,  15, -10,  15, -15,  15, -15,   5,  -5,   5;
             0,  10,  15, -10, -15,   5,  -5, -25,  25,  35;
             0,   0,   0,   0,   0,   0,   0,   0,   0,   0 ...
        ];

        for j = 1:Sims.M
            Tracker(i).q1_hat(:,:,j) = q1_hat_values(:,j);

            Tracker(i).q2_hat(:,:,j) = [-0.1; 0.4; 0];  
            Tracker(i).q_hat(:,:,j) = [Tracker(i).q1_hat(:,:,j); Tracker(i).q2_hat(:,:,j)];  

            Tracker(i).psi_hat_old(j) = -0.1; 
            Tracker(i).v_hat_old(j) = 0.4; 

            Tracker(i).P_hat(:,:,j) = diag([100, 100, 100, 1, 1, 0.1]);  
            Tracker(i).P_save(:,:,1,j) = Tracker(i).P_hat(:,:,j);  

            Tracker(i).alpha = [];
        end
        % Centroid
        Tracker(i).q_hat_bar = mean(Tracker(i).q_hat,3);
    end

    Gains.Q = 1e-4*diag([10 10 10 10 1 0.1]);
    Gains.W = inv(Gains.Q);
    Gains.R = 1; 
    Gains.V = inv(Gains.R);
    Gains.Wt = 1e-1*diag([5 5 5 5 1 1]);
    Gains.pi_max = 5;

    % ----------------------------------- Cooperative Pursuit Parameter------------------------------------------------------

    % Set type of tracking controller 
    epsilon = -1*[1; 1; 0];
    Delta = [1  0          -epsilon(3)  epsilon(2); 
             0  epsilon(3)  0          -epsilon(1);
             0 -epsilon(2)  epsilon(1)  0        ];

    Gains.epsilon = epsilon;
    Gains.Delta_bar = Delta'/(Delta*Delta');   
    Gains.K = 1*diag([.5,.5,.5]);   
    Gains.kz = 200;         
    Gains.kc = 0.1;
    Gains.omega_bar = 0.1;
    Gains.r = 80;
    Gains.zeta = 20;
    Gains.gamma_ddot_max =  0.01;
    Gains.gamma_ddot_min = -0.01;

    for i = 1:Sims.N                            
        Tracker(i).E_Pursuit = [];                % Store pusuit errors
        Tracker(i).pd = [];
        Tracker(i).pd_dot = []; 
        Tracker(i).pd_ddot = [];                                                            % derivative of pd
        Tracker(i).Ep_norm = [];                          
        Tracker(i).omega = [];
        Tracker(i).vc = [];     
        Tracker(i).Com_Control = [];           
        Tracker(i).Com_DEKF = [];
        Tracker(i).gamma_hat = 0;
        Tracker(i).gamma_tilde = [];

        Tracker(i).E_Loc = [];  
        Tracker(i).range = [];

        Tracker(i).z_tilde = [];                                      % will be transmited to neighbor, i.e. Tracker 3       = z_tilde;
        Tracker(i).Omega_tilde = [];   

        Tracker(i).z_cor = [];
        Tracker(i).Omega_cor = [];
        Tracker(i).q_hat_cor = [];

        Tracker(i).E_Loc = [];
        Tracker(i).E_LocM = [];
    end

    % ----------------------------------- Additional Parameter ------------------------------------------------------

    for j = 1:Sims.M
        Target(j).q1 = [];  % target1 position
        Target(j).q2 = [];  % target1 velocity
        Target(j).q = [];  % target1 velocity
    end    
    Targets.q1_bar = [];  % target1 position
    Targets.q2_bar = [];  % target1 velocity
    Targets.q_bar = [];  % target1 velocity

    Sims.ktime = [];    
    Sims.event = [];

    Sims.time = [];                                                              % Store Sims.time
    Sims.time_d = [];                                                            % Store descrete Sims.time

    Sims.DEKF_Times = [];
    Sims.Pursuit_Times = [];

%------------------------------Start simulation ---------------------------

    for t = 0:Sims.ts:Sims.tf
        Sims.time(end+1) = t;   
        if rem(t,2)==0
            Sims.time_d(end+1) = t/2; 
        end
        
        if strcmp(Sims.TargetMotion, 'sync')
            offsets = [  0,  10, -10,  10, -10,  20, -20,   0,   0,   0;
                         0,  10,  10, -10, -10,   0,   0, -20,  20,  30 ];
            for j = 1:Sims.M
                Target(j).q1(:,end+1) = [30*sin(0.01*t) + offsets(1,j);  0.1*t + offsets(2,j);  0]; 
                Target(j).q2(:,end+1) = [atan2(0.1,0.3*cos(0.01*t)); norm([0.3*cos(0.01*t);0.1]); (0.0003*sin(0.01*t))/(0.09*cos(0.01*t)^2+0.01)];
                Target(j).q(:,end+1) = [Target(j).q1(:,end); Target(j).q2(:,end)];
            end
        end
        
        if strcmp(Sims.TargetMotion, 'spread')
            % update target1 position, velocity, and state GREEN - RED
            Target(1).q1(:,end+1) = [30*sin(0.01*t);   0.1*t;   0 ];  % target1 position
            Target(1).q2(:,end+1) = [atan2(0.1,0.3*cos(0.01*t))  ; norm([0.3*cos(0.01*t);0.1]);    (0.0003*sin(0.01*t))/(0.09*cos(0.01*t)^2+0.01) ];  % target1 velocity
            Target(1).q(:,end+1) = [Target(1).q1(:,end); Target(1).q2(:,end)];          % target1 state
            % update target2 position, velocity, and state CYAN - BLACK
            Target(2).q1(:,end+1) = [20*sin(0.01*t) + 10;   0.15*t + 10;   0 ];  % target2 position
            Target(2).q2(:,end+1) = [atan2(0.15,0.2*cos(0.01*t))  ; norm([0.2*cos(0.01*t);0.15]);  (0.0003*sin(0.01*t))/(0.09*cos(0.01*t)^2+0.01) ];  % target1 velocity ];         % target2 velocity (same as Targets.v1)
            Target(2).q(:,end+1) = [Target(2).q1(:,end); Target(2).q2(:,end)];                    % target2 state
            % update target3 position, velocity, and state MAGENTA - YELLOW
            Target(3).q1(:,end+1) = [-30*sin(0.01*t) - 10;   0.1*t + 10;   0 ]; % target3 position
            Target(3).q2(:,end+1) = [atan2(0.1,-0.3*cos(0.01*t)); norm([-0.3*cos(0.01*t);0.1]);   (0.0003*sin(0.01*t))/(0.09*cos(0.01*t)^2+0.01) ];          % target3 velocity (same as Targets.v1)
            Target(3).q(:,end+1) = [Target(3).q1(:,end); Target(3).q2(:,end)];               
        end
        
        if strcmp(Sims.TargetMotion, 'comp')
            % update target1 position, velocity, and state GREEN - RED
            Target(1).q1(:,end+1) = [30*sin(0.01*t);   0.1*t;   0 ];  % target1 position
            Target(1).q2(:,end+1) = [atan2(0.1,0.3*cos(0.01*t)) ; norm([0.3*cos(0.01*t);0.1]);    (0.0003*sin(0.01*t))/(0.09*cos(0.01*t)^2+0.01) ];  % target1 velocity
            Target(1).q(:,end+1) = [Target(1).q1(:,end); Target(1).q2(:,end)];          % target1 state
            % update target2 position, velocity, and state CYAN - BLACK
            Target(2).q1(:,end+1) = [30*sin(0.01*t)+10;   0.1*t+10;   0 ];  % target2 position
            Target(2).q2(:,end+1) = [atan2(0.1,0.3*cos(0.01*t)) ; norm([0.3*cos(0.01*t);0.1]);    (0.0003*sin(0.01*t))/(0.09*cos(0.01*t)^2+0.01) ];  % target1 velocity ];         % target2 velocity (same as Targets.v1)
            Target(2).q(:,end+1) = [Target(2).q1(:,end); Target(2).q2(:,end)];                    % target2 state
            % update target3 position, velocity, and state MAGENTA - YELLOW
            Target(3).q1(:,end+1) = [-0.05*t - 10;   0.05*t + 10;   0 ]; % target3 position
            Target(3).q2(:,end+1) = [atan2(0.05,-0.05); norm([-0.05;0.05]); 0];          % target3 velocity (same as Targets.v1)
            Target(3).q(:,end+1) = [Target(3).q1(:,end); Target(3).q2(:,end)];                  % target2 state
            % update target3 position, velocity, and state MAGENTA - YELLOW
            Target(4).q1(:,end+1) = [0.1*t + 10;   -10;   0 ]; % target3 position
            Target(4).q2(:,end+1) = [0;0.1;0];          % target3 velocity (same as Targets.v1)
            Target(4).q(:,end+1) = [Target(4).q1(:,end); Target(4).q2(:,end)];                  % target2 state
            % update target3 position, velocity, and state MAGENTA - YELLOW
            Target(5).q1(:,end+1) = [-30*sin(0.01*t) - 10;   30*cos(0.01*t) - 40;   0 ]; % target3 position
            Target(5).q2(:,end+1) = [atan2(-0.3*sin(0.01*t),-0.3*cos(0.01*t)); norm([-0.3*cos(0.01*t);-0.3*sin(0.01*t)]); (1/30)*cos(0.02*t)];          % target3 velocity (same as Targets.v1)
            Target(5).q(:,end+1) = [Target(5).q1(:,end); Target(5).q2(:,end)];   
        end
        
        % centroid
        Targets.q1_bar(:,end+1) = mean(cell2mat(arrayfun(@(x) x.q1(:, end), Target, 'UniformOutput', false)),2);
        Targets.q2_bar(:,end+1) = mean(cell2mat(arrayfun(@(x) x.q2(:, end), Target, 'UniformOutput', false)),2);
        Targets.q_bar(:,end+1) = [Targets.q1_bar(:,end); Targets.q2_bar(:,end)];

        % Update r
        norms = [];
        for j = 1:Sims.M
            norms(:,end+1) = norm(Targets.q1_bar(:,end) - Target(j).q1(:,end));
        end
        Gains.r = max(norms)+Gains.zeta;

        
        tic;
        % compute correction speeds using the coordination controller 
        if ~isempty(Sims.event) && Sims.event(end) == 1
            [Tracker] = CoordinationCC(Tracker, Gains.kc,Sims.N);     
        else
            for i = 1:Sims.N 
                Tracker(i).vc = 0;
            end
        end

        for i = 1:Sims.N
            Tracker(i).omega(end+1) = Gains.omega_bar + Tracker(i).vc(end);
            Tracker(i) = ST_Tracking(Tracker(i), Targets, t, Sims.ts, Sims.TTC_type, Gains, Sims.ts);
        end
        Sims.Pursuit_Times(end+1) = toc;
        
        for i = 1:Sims.N
            for j = 1:Sims.M
                Tracker(i).range(j) = (1+0.004*randn)*norm(Tracker(i).p1(:,end) - Target(j).q1(:,end));
            end
        end

        tic;
        if strcmp(Sims.ET_Type, 'ET')  
            [Tracker, Sims] = EventTriggeredDEKF(Tracker,Sims.ts,t,Sims,Gains);     % go with periodic communications
        else
            [Tracker, Sims] = DEKF(Tracker,Sims.ts,t,Sims,Gains);     % go with periodic communications
        end
        Sims.DEKF_Times(end+1) = toc;
        
        for i = 1:Sims.N            
            Tracker(i).E_LocM(:,end+1,:) = zeros(3,1,Sims.M);   
            Tracker(i).E_Loc(end+1,:) = zeros(1,Sims.M);
            for j = 1:Sims.M
                Tracker(i).q1_hat(:,end,j) = Tracker(i).q_hat(1:3,end,j);
                Tracker(i).q2_hat(:,end,j) = Tracker(i).q_hat(4:6,end,j);
                
                Tracker(i).E_LocM(:,end,j) = Target(j).q1(:,end) - Tracker(i).q1_hat(:,end,j);   
                Tracker(i).E_Loc(end,j) = norm(Tracker(i).E_LocM(:,end,j));   
            end
            Tracker(i).Ep_norm(end+1) = norm(Tracker(i).E_Pursuit(:)); 
            
            Tracker(i).p(:,end+1) = update_vehicle(Tracker(i).p(:,end),Tracker(i).Input(:,end),Sims.ts,t);  
            Tracker(i).p1 = Tracker(i).p(1:3,:);
            Tracker(i).p2 = Tracker(i).p(4:6,:);

            Tracker(i).gamma(end+1) = Tracker(i).gamma(end) + Sims.ts*Tracker(i).gamma_dot(end);   
        end
    end
    plot(Tracker(i).E_Loc(:,1));

    Sims.comm = sum(Sims.event)/length(Sims.event);
    
    E_Pursuit = vertcat(Tracker(:).E_Pursuit);  % Combine all E_Pursuit
    E_Pursuit = reshape(E_Pursuit(:,1000:end), 1, []);
    Sims.RMSE_Pursuit = sqrt(mean(E_Pursuit.^2));
    
    E_Loc = reshape([Tracker.E_Loc], 1, []);
    Sims.RMSE_Locs = sqrt(mean(E_Loc.^2,2));
    
    Sims.Pursuit_Time_Mean = mean(Sims.Pursuit_Times);
    Sims.DEKF_Time_Mean = mean(Sims.DEKF_Times);
    
    save(['Datas/' Sims.name '.mat']);
    save_to_base(1);
    
    % CSV
    filename = 'Datas/output.csv'; % Define the CSV file
    new_data = {Sims.name, Sims.N, Sims.M, Sims.ET_Type, Sims.TargetMotion, Sims.comm, Sims.RMSE_Pursuit, Sims.RMSE_Locs, Sims.Pursuit_Time_Mean, Sims.DEKF_Time_Mean};

    % Read existing file if it exists
    if isfile(filename)
        existing_data = readcell(filename);
    else
        % Initialize with headers if the file does not exist
        existing_data = {"Name", "N", "M", "ETtype", "MotionType", "Comm", ...
                         "RMSE_Pursuit", "RMSE_Loc", "Pursuit_Time_Mean", "DEKF_Time_Mean"};
    end

    if strcmp(Sims.runLog, 'true')
        % Convert to table for easier manipulation
        variable_names = cellstr(existing_data(1, :)); % Convert to character vectors
        data_table = cell2table(existing_data(2:end, :), 'VariableNames', variable_names);

        name = new_data{1};
        values = new_data{2:end};
        idx = find(strcmp(data_table.Name, name));

        if isempty(idx)
            % Append new row if name does not exist
            data_table = [data_table; cell2table(new_data, 'VariableNames', data_table.Properties.VariableNames)];
        else
            % Replace existing row if name is found
            data_table{idx, 2:end} = values;
        end
        % Write back to CSV
        disp(new_data)
        writetable(data_table, filename);
    end
end

%% ====================== Auxilary functions =============================
function p_next=update_vehicle(p_current,input,ts,t)
    input_robot.nSteps = 4;
    input_robot.ts=ts;
    input_robot.u=input;
    input_robot.p = p_current;
    output_robot = RK4_integrator(@vehicle, input_robot);
    p_next = output_robot.value;
end

%% vehicle dynamics 3D
function dx = vehicle(t,x,u)
phi=x(4);
theta=x(5);
psi=x(6);

v=u(1);
p=u(2);
q=u(3);
r=u(4);

dx=[v*cos(psi)*cos(theta);
    v*sin(psi)*cos(theta);
   -v*sin(theta);
    p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta); 
    q*cos(phi)-r*sin(phi); 
    q*sin(phi)/cos(theta)+r*cos(phi)/cos(theta)]; 
end