clear all;
clear; clc;
% ----------------------------------- Simulation Initialization ------------------------------------------------------

Sims.tf = 400;                              % simulation time  
Sims.ts = 0.1;                              % simulation time interval  
Sims.T_meas = 2;                            % measurement time interval 

Sims.N = 1;                                 % number of tracker used
Sims.M = 3;                                 % number of target used

Sims.TargetMotion = 'sync';                 % 'sync', 'spread', 'comp'
Sims.ET_Type = 'ET';                        % 'ET', 'noET'
Sims.runLog = 'false';     % 'true', 'false'
Sims.runPlot = 'true';          % 'true', 'false'
Sims.runAnimation = 'false';     % 'true', 'false'
Sims.TTC_type = 'TypeIII';

% do not change
% if Sims.N == 1, Sims.ET_Type = 'noET'; end   
if strcmp(Sims.TargetMotion, 'spread'), Sims.M = 3; end
if strcmp(Sims.TargetMotion, 'comp'), Sims.M = 5; end

Sims.name = sprintf('%dN%dM_%s_%s_%s', Sims.N,Sims.M,Sims.TargetMotion,Sims.ET_Type,Sims.TTC_type);
SLAP(Sims);

% Plot and Animate
if strcmp(Sims.runPlot, 'true'), Plot(); end
if strcmp(Sims.runAnimation, 'true'), Animation(); end
Sims.RMSE_Pursuit
beep;






































%% for loop for many sims
for k = 1:1
    clc; 
    clearvars -except k;  % Preserve k and other necessary variables
    
    Sims.N = k;                                % number of tracker used
    Sims.M = 3;                               % number of target used
    if k==1, Sims.N = 1; end
    
    Sims.tf = 400;                             % simulation time  
    Sims.ts = 0.1;                             % simulation time interval  
    Sims.T_meas = 2;                           % measurement time interval 

    Sims.TargetMotion = 'spread';     % 'sync', 'spread', 'comp'
    Sims.ET_Type = 'ET';            % 'ET', 'noET'
    Sims.runLog = 'false';     % 'true', 'false'
    Sims.runPlot = 'true';          % 'true', 'false'
    Sims.runAnimation = 'false';     % 'true', 'false'
    Sims.TTC_type = 'TypeI';

    % do not change
%     if Sims.N == 1, Sims.ET_Type = 'noET'; end   
    if strcmp(Sims.TargetMotion, 'spread'), Sims.M = 3; end
    if strcmp(Sims.TargetMotion, 'comp'), Sims.M = 5; end

    Sims.name = sprintf('%dN%dM_%s_%s_%s', Sims.N,Sims.M,Sims.TargetMotion,Sims.ET_Type,Sims.TTC_type);
    SLAP(Sims);
    
    % Plot and Animate
    if strcmp(Sims.runPlot, 'true'), Plot(); end
    if strcmp(Sims.runAnimation, 'true'), Animation(); end
    Sims.RMSE_Pursuit
    beep;
end
beep;

%%
for k = 1:1
    clearvars -except k;  % Preserve k and other necessary variables
    
    Sims.N = 4;                                % number of tracker used
    Sims.M = 5;                               % number of target used
    if k==1, Sims.N = 1; end
    
    Sims.tf = 400;                             % simulation time  
    Sims.ts = 0.1;                             % simulation time interval  
    Sims.T_meas = 2;                           % measurement time interval 

    Sims.TargetMotion = 'comb';     % 'sync', 'spread', 'comb'
    Sims.ET_Type = 'ET';            % 'ET', 'noET'
    Sims.TTC_type = 'TypeI';

    % do not change
%     if Sims.N == 1, Sims.ET_Type = 'noET'; end   
    if strcmp(Sims.TargetMotion, 'spread'), Sims.M = 3; end
    if strcmp(Sims.TargetMotion, 'comp'), Sims.M = 5; end

    filename = sprintf('Datas/%dUAV%dTARGET_%s_%s_%s.mat', Sims.N,Sims.M,Sims.TargetMotion,Sims.ET_Type,Sims.TTC_type);

    load(filename);
%     k
%     Sims.RMSE_Pursuit
%     value = {Sims.N, Sims.M, Sims.comm, Sims.RMSE_Pursuit, Sims.RMSE_Locs, Sims.Pursuit_Time_Mean, Sims.DEKF_Time_Mean}

    
    Sims.runPlot = 'true';          % 'true', 'false'
    Sims.runAnimation = 'false';     % 'true', 'false'

    % Plot and Animate
    if strcmp(Sims.runPlot, 'true'), Plot(); end
    if strcmp(Sims.runAnimation, 'true'), Animation(); end
end
beep;