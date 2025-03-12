close all;

time = Sims.time;
ktime = Sims.ktime;

AUV_COL= [0 0 0];
AUV_Color(3,:) = [0.8 0.8 0]; % yellow
AUV_Color(2,:) = [0.1 0.1 0]; % yellow
AUV_Color(1,:) = [0.9 0.2 0]; % yellow
AUV_Color(4,:) = [0.2314 0.6745 0.9451];  
AUV_Color(5,:) = [0.8549 0.2431 0.5373];  
AUV_Color(6,:) = [0.1294 0.8431 0.3647];  
AUV_Color(7,:) = [0.7686 0.6196 0.2196];  
AUV_Color(8,:) = [0.4157 0.3804 0.8039];  
AUV_Color(9,:) = [0.9608 0.4863 0.1451];  

TARGET_Color(1,:) = [0.0 1.0 1.0];   % Cyan
TARGET_Color(2,:) = [1.0 0.0 1.0];   % Magenta
TARGET_Color(3,:) = [0.5 0.0 0.5];   % Purple
TARGET_Color(4,:) = [0.3 0.3 0.3];   % Dark Gray
TARGET_Color(5,:) = [0.2 0.9 0.7];   % Light Gray
TARGET_Color(6,:) = [1.0 0.0 0.0];   % Red
TARGET_Color(7,:) = [0.0 1.0 0.0];   % Green
TARGET_Color(8,:) = [0.9 0.5 0.0];   % Orange
TARGET_Color(9,:) = [0.0 0.0 1.0];   % Blue
TARGET_Color(10,:) = [1.0 1.0 0.0];  % Yellow
TARGETs_Color = [0.2314 0.6745 0.9451];  % Light Blue

VEL_color = [0.8500, 0.3250, 0.0980];
ORIEN_color(1,:) = [0.4940, 0.1840, 0.5560];
ORIEN_color(2,:) = [0.3010, 0.7450, 0.9330];
ORIEN_color(3,:) = [0.6350, 0.0780, 0.1840];

%% Target Estimation error 

L = length(time);
h = [];
legendStr = {};

fig1 = figure(1);
set(fig1, 'position', [0 0 800 200*Sims.M]); % Adjust height for multiple subplots

for j = 1:Sims.M
    subplot(Sims.M,1,j);
    hold on;
    for i = 1:Sims.N
        h(i,j) = plot(time(1:L), Tracker(i).E_Loc(1:L,j), '-', 'LineWidth', 2, 'Color', AUV_Color (i,:));        
        legendStr{i} = sprintf('$||\\tilde{\\mathbf{q}}_1^{%d,%d}||$', i, j);
    end
    hold off;
    
    xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('Errors(m)', 'FontSize', 12, 'Interpreter', 'latex');
    title(sprintf('Target %d', j), 'FontSize', 14, 'Interpreter', 'latex');
    
    % Add legend with dynamic strings
    lgd(j) = legend(h(:,j), legendStr{:}, 'FontSize', 12, 'Interpreter', 'latex');
    ylim([0 5]);
end
saveas(fig1, ['Figures/' Sims.name '_eest.png']);
savefig(fig1, ['Figures/' Sims.name '_eest.fig']); % Save as .fig

%% Target State Covariance Alpha
Lk = length(ktime);
h = [];
legendStr = {};

fig2 = figure(2);
set(fig2, 'position', [0 0 800 200*Sims.M]); % Adjust height for multiple subplots
for j = 1:Sims.M
    subplot(Sims.M,1,j);
    hold on;
    for i = 1:Sims.N
        h(i,j) = plot(ktime(1:Lk), Tracker(i).alpha(1:Lk,j), '-', 'LineWidth', 2, 'Color', AUV_Color (i,:));
        legendStr{i} = sprintf('$\\alpha^{%d,%d}$', i, j);
    end
    if strcmp(Sims.ET_Type, 'ET')
        h(i+1,j) = stem(ktime(1:Lk), Gains.pi_max*Sims.event(1:Lk), 'b-*', 'LineWidth', 1);
        legendStr{i+1} = 'Event';
        h(i+2,j) = plot(ktime(1:Lk), Gains.pi_max*ones(1,Lk), 'g--','LineWidth',1 );
        legendStr{i+2} = '$\Pi$';
    end

    hold off;
    
    xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('Alpha', 'FontSize', 12, 'Interpreter', 'latex');
    title(sprintf('Target %d', j), 'FontSize', 14, 'Interpreter', 'latex');
    
    % Add legend with dynamic strings
    lgd(j) = legend(h(:, j), legendStr{:}, 'FontSize', 12, 'Interpreter', 'latex');
    ylim([0 10]);
end
saveas(fig2, ['Figures/' Sims.name '_alpha.png']);
savefig(fig2, ['Figures/' Sims.name '_alpha.fig']); % Save as .fig

%% Tracker Gamma
L = length(time);
h = [];
legendStr = {};

fig3 = figure(3);
set(fig3, 'position', [0 0 800 200]); % Adjust height for multiple subplots
hold on;
for i = 1:Sims.N
    h(i)=plot(time,Tracker(i).gamma(1:end-1),'LineWidth',2,'Color',AUV_Color(i,:));
    legendStr{i} = sprintf('$\\gamma^{%d}$', i);
end
hold off;
xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('Gamma', 'FontSize', 12, 'Interpreter', 'latex');
title('Tracker', 'FontSize', 14, 'Interpreter', 'latex');

% Add legend with dynamic strings
lgd = legend(h, legendStr, 'FontSize', 12, 'Interpreter', 'latex');
ylim([0 100]);

saveas(fig3, ['Figures/' Sims.name '_gamma.png']);
savefig(fig3, ['Figures/' Sims.name '_gamma.fig']); % Save as .fig

%% Tracker E Pursuit
L = length(time);
h = [];
legendStr = {};

fig4 = figure(4);
set(fig4, 'position', [0 0 800 200]); % Adjust height for multiple subplots
hold on;
for i = 1:Sims.N
    h(i)=plot(time,Tracker(i).E_Pursuit,'LineWidth',2,'Color',AUV_Color(i,:));
    legendStr{i} = sprintf('$||\\bf{e}_\\mathrm{p}^%d||$', i);
end
hold off;
xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('E. Pursuit', 'FontSize', 12, 'Interpreter', 'latex');
title('Tracker', 'FontSize', 14, 'Interpreter', 'latex');

% Add legend with dynamic strings
lgd = legend(h, legendStr, 'FontSize', 12, 'Interpreter', 'latex');
ylim([0 100]);

saveas(fig4, ['Figures/' Sims.name '_epursuit.png']);
savefig(fig4, ['Figures/' Sims.name '_epursuit.fig']); % Save as .fig

%% Tracker Input
Lk = length(ktime);
h = [];
legendStr = {};

fig5 = figure(5);
set(fig5, 'position', [0 0 800 300*Sims.N]); % Adjust height for multiple subplots
for i = 1:Sims.N
    subplot(2*Sims.N,1,2*i-1);
    h = plot(time, Tracker(i).Input(1,2:end), '-', 'LineWidth', 2, 'Color', VEL_color);
    legend(h, sprintf('$v_\\mathbf{x}^%d$', i), 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('$u_1$[m/s]', 'FontSize', 12, 'Interpreter', 'latex');
    title(sprintf('Tracker %d', i), 'FontSize', 14, 'Interpreter', 'latex');
    ylim([0 10]);

    subplot(2*Sims.N,1,2*i);
    hold on;
    h1 = plot(time, Tracker(i).Input(2,2:end), '-', 'LineWidth', 2, 'Color', ORIEN_color(1,:));
    h2 = plot(time, Tracker(i).Input(3,2:end), '-', 'LineWidth', 2, 'Color', ORIEN_color(2,:));
    h3 = plot(time, Tracker(i).Input(4,2:end), '-', 'LineWidth', 2, 'Color', ORIEN_color(3,:));
    hold off;
    legend([h1 h2 h3], sprintf('$p^%d$', i), sprintf('$q^%d$', i), sprintf('$r^%d$', i), 'FontSize', 12, 'Interpreter', 'latex');
    xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('$u_2$[rad/s]', 'FontSize', 12, 'Interpreter', 'latex');
    ylim([-1 1]);
end
saveas(fig5, ['Figures/' Sims.name '_input.png']);
savefig(fig5, ['Figures/' Sims.name '_input.fig']); % Save as .fig

%% Tracker Gamma Dot
L = length(time);
h = [];
legendStr = {};

fig6 = figure(6);
set(fig6, 'position', [0 0 800 200]); % Adjust height for multiple subplots
hold on;
for i = 1:Sims.N
    h(i)=plot(time,Tracker(i).gamma_dot(1:end-1),'LineWidth',2,'Color',AUV_Color(i,:));
    legendStr{i} = sprintf('$\\dot{\\gamma}^{%d}$', i);
end
hold off;
xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('Gamma Dot', 'FontSize', 12, 'Interpreter', 'latex');
title('Tracker', 'FontSize', 14, 'Interpreter', 'latex');

% Add legend with dynamic strings
lgd = legend(h, legendStr, 'FontSize', 12, 'Interpreter', 'latex');

saveas(fig6, ['Figures/' Sims.name '_gammadot.png']);
savefig(fig6, ['Figures/' Sims.name '_gammadot.fig']); % Save as .fig

%% Track Target
N = Sims.N;
M = Sims.M;
h = [];
legendStr = {};

fig7=figure(7);
hold on;

for j = 1:M
    % Targets Path
    h(1,j) = plot(Target(j).q1(1,:), Target(j).q1(2,:), '-', 'LineWidth', 2, 'Color', TARGET_Color(j,:));
    legendStr{1,j} = sprintf('${\\bf q}^{%d}$', j);
    for i = 1:N
        % Estimated position
        h(2,(j-1)*N+i) = plot(Tracker(i).q_hat(1,:,j), Tracker(i).q_hat(2,:,j), '*', 'MarkerSize', 2, 'LineWidth', 0.2, 'Color', AUV_Color(i,:));
        legendStr{2,i} = sprintf('$\\hat{\\bf{q}}^{%d,j}$', i);
    end
end
% Centroid path
h(1,M+1) = plot(Targets.q_bar(1,:), Targets.q_bar(2,:), '-.', 'LineWidth', 1, 'Color', 'b');
legendStr{1,M+1} = sprintf('$\\bar{\\bf{q}}$');
hold off;

xlabel('{\bf x}[m]', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('{\bf y}[m]', 'FontSize', 12, 'Interpreter', 'latex');
ah1=axes('position',get(gca,'position'),'visible','off');
lgd1 = legend(h(1,1:M+1), legendStr{1,1:M+1}, 'FontSize', 12, 'Interpreter', 'latex');
set(lgd1, 'Location', 'northwest'); % Position the first legend
lgd2 = legend(ah1,h(2,1:N), legendStr{2,1:N}, 'FontSize', 12, 'Interpreter', 'latex');
set(lgd2, 'Location', 'northeast'); % Position the second legend

% % Temporary Triangle lines
% i=0;
% tem1=[];
% tem2=[];
% tem3=[];
% while i<length(Tracker1.pd(:,1))
%     if (rem(i-1,500)==0)
%         tem1 = [tem1, [[Targets.q1(i,:)'], [Targets.q2(i,:)']]]; 
%         tem2 = [tem2, [[Targets.q2(i,:)'], [Targets.q3(i,:)']]];
%         tem3 = [tem3, [[Targets.q3(i,:)'], [Targets.q1(i,:)']]];
%     end
%     i=i+1;
% end
% 
% for j = 1:size(tem1, 2)/2
%     h8=plot(tem1(1,j*2-1:j*2), tem1(2,j*2-1:j*2), 'Color', [0.5, 0.5, 0.5], 'LineStyle', '-.', 'LineWidth', 1);
%     h9=plot(tem2(1,j*2-1:j*2), tem2(2,j*2-1:j*2), 'Color', [0.5, 0.5, 0.5], 'LineStyle', '-.', 'LineWidth', 1);
%     h10=plot(tem3(1,j*2-1:j*2), tem3(2,j*2-1:j*2), 'Color', [0.5, 0.5, 0.5], 'LineStyle', '-.', 'LineWidth', 1);
% end

% Set the plot view to 2D
axis equal;
grid on;

saveas(fig7, ['figures/' Sims.name '_track.png']);
savefig(fig7, ['figures/' Sims.name '_track.fig']); % Save as .fig

%% Simulation

fig8=figure(8);
N1 = 100;   % Tracker path trail length
N2 = 1000;  % Pd trail length
N3 = 500;

set(fig8, 'Units', 'normalized', 'Position', [0,0,1,1]);

h = [];
legendStr = {};

hold on;
    
for i = 1:Sims.N
    % Tracker path
    h(Sims.M+Sims.N+1+i) = plot3(Tracker(i).p1(1,end-N1:end),Tracker(i).p1(2,end-N1:end),Tracker(i).p1(3,end-N1:end),'-','LineWidth',2,'Color',AUV_Color(i,:));
    legendStr{Sims.M+Sims.N+1+i} = sprintf('${\\bf p}^{%d}$', i);
    
    % Tracker plane
    GTF_Plane([Tracker(i).p1(1,end),Tracker(i).p1(2,end),Tracker(i).p1(3,end)], [Tracker(i).p2(1,end)*180/pi+180,Tracker(i).p2(2,end)*180/pi,Tracker(i).p2(3,end)*180/pi+180], 1.2, 0, AUV_Color(i,:),1)

    % Pd
    h(Sims.M+2*Sims.N+1+i) = plot3(Tracker(i).pd(1,end-N2:end),Tracker(i).pd(2,end-N2:end),Tracker(i).pd(3,end-N2:end),'-.','LineWidth',0.2,'Color',AUV_Color(i,:));
    legendStr{Sims.M+Sims.N+1+i} = sprintf('${\\bf P}_d^{%d}$', i);

    % Estimated
    for j = 1:Sims.M
        h(Sims.M+1+i) = plot3(Tracker(i).q_hat(1,1:end,j),Tracker(i).q_hat(2,1:end,j),Tracker(i).q_hat(3,1:end,j),'*','LineWidth',0.2,'Color',AUV_Color(i,:));
        legendStr{Sims.M+1+i} = sprintf('$\\hat{\\bf{q}}^{%d,j}$', i);
    end
end

for j = 1:Sims.M
    % Target path
    h(j) = plot3(Target(j).q1(1,1:end),Target(j).q1(2,1:end),Target(j).q1(3,1:end),'-','LineWidth',4,'Color', TARGET_Color(j,:));
    legendStr{j} = sprintf('${\\bf q}^{%d}$', j);
    
    % Target ship
    GTF_Ship([Target(j).q1(1,end),Target(j).q1(2,end),Target(j).q1(3,end)], Target(j).q2(1,end)*180/pi+180, 2, 0, TARGET_Color(j,:),1)
end
h(Sims.M+1) = plot3(Targets.q1_bar(1,1:end),Targets.q1_bar(2,1:end),Targets.q1_bar(3,1:end),'--', 'LineWidth', 2, 'Color', TARGETs_Color);
legendStr{Sims.M+1} = sprintf('$\\bar{\\bf{q}}$');

view(201,17);
hold off;
grid on;

lgd = legend(h, legendStr, 'FontSize', 14, 'Interpreter', 'latex');
lgd.Location = 'east';  % Change location

title(sprintf('Simulation of %s', Sims.name), 'FontSize', 14, 'Interpreter', 'latex');
xlabel('X[m]')
ylabel('Y[m]')
zlabel('Z[m]')

saveas(fig8, ['figures/' Sims.name '_sim.png']);
savefig(fig8, ['figures/' Sims.name '_sim.fig']); % Save as .fig

%% Track Tracker
N = Sims.N;
M = Sims.M;
h = [];
legendStr = {};

fig10=figure(10);
hold on;

for i = 1:N
%    Targets Path
    h(1,i) = plot(Tracker(i).p1(1,:), Tracker(i).p1(2,:), '-', 'LineWidth', 2, 'Color', AUV_Color(i,:));
    legendStr{1,i} = sprintf('${\\bf p}^{%d}$', i);
%     
    h(2,i) = plot(Tracker(i).pd(1,:), Tracker(i).pd(2,:), '--', 'LineWidth', 1, 'Color', AUV_Color(i,:));
    legendStr{2,i} = sprintf('${\\bf p}_d^{%d}$', i);
    
end

xlabel('{\bf x}[m]', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('{\bf y}[m]', 'FontSize', 12, 'Interpreter', 'latex');
% lgd = legend(h,legendStr, 'FontSize', 12, 'Interpreter', 'latex');
% set(lgd, 'Location', 'northeast'); % Position the second legend

% % Temporary Triangle lines
% i=0;
% tem1=[];
% tem2=[];
% tem3=[];
% while i<length(Tracker1.pd(:,1))
%     if (rem(i-1,500)==0)
%         tem1 = [tem1, [[Targets.q1(i,:)'], [Targets.q2(i,:)']]]; 
%         tem2 = [tem2, [[Targets.q2(i,:)'], [Targets.q3(i,:)']]];
%         tem3 = [tem3, [[Targets.q3(i,:)'], [Targets.q1(i,:)']]];
%     end
%     i=i+1;
% end
% 
% for j = 1:size(tem1, 2)/2
%     h8=plot(tem1(1,j*2-1:j*2), tem1(2,j*2-1:j*2), 'Color', [0.5, 0.5, 0.5], 'LineStyle', '-.', 'LineWidth', 1);
%     h9=plot(tem2(1,j*2-1:j*2), tem2(2,j*2-1:j*2), 'Color', [0.5, 0.5, 0.5], 'LineStyle', '-.', 'LineWidth', 1);
%     h10=plot(tem3(1,j*2-1:j*2), tem3(2,j*2-1:j*2), 'Color', [0.5, 0.5, 0.5], 'LineStyle', '-.', 'LineWidth', 1);
% end

% Set the plot view to 2D
axis equal;
grid on;

saveas(fig10, ['figures/' Sims.name '_trackers.png']);
savefig(fig10, ['figures/' Sims.name '_trackers.fig']); % Save as .fig