close all;

time = Sims.time;
ktime = Sims.ktime;

v = VideoWriter(['Videos/' Sims.name '_video']);
v.FrameRate=5;
open(v);
fig9=figure(9);

%colors
TARGET_Color(1,:) = [0.0 1.0 1.0];   % Cyan
TARGET_Color(2,:) = [1.0 0.0 1.0];   % Magenta
TARGET_Color(3,:) = [0.5 0.0 0.5];   % Purple
TARGET_Color(4,:) = [0.3 0.3 0.3];   % Dark Gray
TARGET_Color(5,:) = [0.2 0.9 0.9];   % Light Gray
TARGET_Color(6,:) = [1.0 0.0 0.0];   % Red
TARGET_Color(7,:) = [0.0 1.0 0.0];   % Green
TARGET_Color(8,:) = [0.9 0.5 0.0];   % Orange
TARGET_Color(9,:) = [0.0 0.0 1.0];   % Blue
TARGET_Color(10,:) = [1.0 1.0 0.0];  % Yellow
TARGETs_Color = [0.2314 0.6745 0.9451];  % Light Blue

AUV_Color(1,:) = [0.8 0.8 0]; % yellow
AUV_Color(2,:) = [0.1 0.1 0]; % yellow
AUV_Color(3,:) = [0.9 0.2 0]; % yellow
AUV_Color(4,:) = [0.2314 0.6745 0.9451];  
AUV_Color(5,:) = [0.8549 0.2431 0.5373];  
AUV_Color(6,:) = [0.1294 0.8431 0.3647];  
AUV_Color(7,:) = [0.7686 0.6196 0.2196];  
AUV_Color(8,:) = [0.4157 0.3804 0.8039];  
AUV_Color(9,:) = [0.9608 0.4863 0.1451];  

VEL_color = [0.8500, 0.3250, 0.0980];
ORIEN_color(1,:) = [0.4940, 0.1840, 0.5560];
ORIEN_color(2,:) = [0.3010, 0.7450, 0.9330];
ORIEN_color(3,:) = [0.6350, 0.0780, 0.1840];

set(fig9, 'Units', 'normalized', 'Position', [0,0,1,1]);

N1 = 100;   % Tracker path trail length
N2 = 1000;  % Pd trail length
N3 = 500;

idx = 1;
i = 1;
j = 1;
k = 1;
while idx < length(time)
    clf;
    figure(fig9);
    
    subplot(4, 3, [1,2,4,5,7,8]); % Row 1 spans two columns
    hold on;
    
    for i = 1:Sims.N
        % Tracker path
        if idx>N1
            plot3(Tracker(i).p1(1,idx-N1:idx),Tracker(i).p1(2,idx-N1:idx),Tracker(i).p1(3,idx-N1:idx),'-','LineWidth',2,'Color',AUV_Color(i,:));
        else
            plot3(Tracker(i).p1(1,1:idx),Tracker(i).p1(2,1:idx),Tracker(i).p1(3,1:idx),'-','LineWidth',2,'Color',AUV_Color(i,:));
        end
        
        % Tracker plane
        GTF_Plane([Tracker(i).p1(1,idx),Tracker(i).p1(2,idx),Tracker(i).p1(3,idx)], [Tracker(i).p2(1,idx)*180/pi+180,Tracker(i).p2(2,idx)*180/pi,Tracker(i).p2(3,idx)*180/pi+180], 1.2, 0, AUV_Color(i,:),1)
    
        % Pd
        if idx>N2
            plot3(Tracker(i).pd(1,idx-N2:idx),Tracker(i).pd(2,idx-N2:idx),Tracker(i).pd(3,idx-N2:idx),'-.','LineWidth',0.2,'Color',AUV_Color(i,:));
        else
            plot3(Tracker(i).pd(1,1:idx),Tracker(i).pd(2,1:idx),Tracker(i).pd(3,1:idx),'-.','LineWidth',0.2,'Color',AUV_Color(i,:));
        end
        
        % Estimated
        for j = 1:Sims.M
            plot3(Tracker(i).q_hat(1,1:idx,j),Tracker(i).q_hat(2,1:idx,j),Tracker(i).q_hat(3,1:idx,j),'*','LineWidth',0.2,'Color',AUV_Color(i,:));
        end
        
    end
    
    for j = 1:Sims.M
        % Target path
        plot3(Target(j).q1(1,1:idx),Target(j).q1(2,1:idx),Target(j).q1(3,1:idx),'-','LineWidth',4,'Color', TARGET_Color(j,:));
        
        % Target ship
        GTF_Ship([Target(j).q1(1,idx),Target(j).q1(2,idx),Target(j).q1(3,idx)], Target(j).q2(1,idx)*180/pi+180, 2, 0, TARGET_Color(j,:),1)
    end
    plot3(Targets.q1_bar(1,1:idx),Targets.q1_bar(2,1:idx),Targets.q1_bar(3,1:idx),'--', 'LineWidth', 2, 'Color', TARGETs_Color);
  
    view(201,17);
    hold off;
    title(sprintf('Simulation of %s', Sims.name), 'FontSize', 14, 'Interpreter', 'latex');
    xlabel('X[m]')
    ylabel('Y[m]')
    zlabel('Z[m]')
    
    subplot(4, 3, 3); % Row 1 spans two columns
    plot(time(1:idx), Tracker(1).Input(1,1:idx), '-', 'LineWidth', 2, 'Color', VEL_color);
    ylabel('$u_1$[m/s]', 'FontSize', 12, 'Interpreter', 'latex');
    title("Tracker 1 Input", 'FontSize', 14, 'Interpreter', 'latex');
    ylim([0 10]);
    if idx>N3
        xlim([(idx-N3)/10 idx/10]);
    else
        xlim([1 N3/10]);
    end   

    subplot(4, 3, 6); % Row 1 spans two columns
    hold on;
    plot(time(1:idx), Tracker(1).Input(2,1:idx), '-', 'LineWidth', 2, 'Color', ORIEN_color(1,:));
    plot(time(1:idx), Tracker(1).Input(3,1:idx), '-', 'LineWidth', 2, 'Color', ORIEN_color(2,:));
    plot(time(1:idx), Tracker(1).Input(4,1:idx), '-', 'LineWidth', 2, 'Color', ORIEN_color(3,:));
    hold off;
    xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('$u_2$[rad/s]', 'FontSize', 12, 'Interpreter', 'latex');
    ylim([-1 1]);
    if idx>N3
        xlim([(idx-N3)/10 idx/10]);
    else
        xlim([1 N3/10]);
    end   
    
    subplot(4, 3, 9); % Row 1 spans two columns
    hold on;
    for i = 1:Sims.N
        plot(time(1:idx),Tracker(i).E_Pursuit(1:idx),'LineWidth',2,'Color',AUV_Color(i,:));
    end
    hold off;
    xlabel('$t(s)$','FontSize',12,'Interpreter','latex');
    ylabel('Errors[m]','FontSize',12,'Interpreter','latex');
    title('Tracker Pursuit Error', 'FontSize', 14, 'Interpreter', 'latex');
    ylim([0 100]);
    if idx>N3
        xlim([(idx-N3)/10 idx/10]);
    else
        xlim([1 N3/10]);
    end   

    subplot(4, 3, 10); % Row 1 spans two columns
    hold on;
    for i = 1:Sims.N
        plot(ktime(1:idx/20), Tracker(i).alpha(1:idx/20,1), '-', 'LineWidth', 2, 'Color', AUV_Color(i,:));
    end
    if strcmp(Sims.ET_Type, 'ET')
        plot(ktime(1:idx/20), Gains.pi_max*ones(1,floor(idx/20)), 'g--','LineWidth',1 );
        stem(ktime(1:idx/20), Gains.pi_max*Sims.event(1:idx/20), 'b-*', 'LineWidth', 1);
    end
    hold off;
    
    xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('Alpha', 'FontSize', 12, 'Interpreter', 'latex');
    title('Target 1  Alpha', 'FontSize', 14, 'Interpreter', 'latex');
%         lgd4 = legend([h43 h44 h1 h100], '$\alpha^{1}$', '$\alpha^{2}$', 'Event', '$\Pi$');
%         set(lgd4, 'FontSize', 12, 'Interpreter', 'latex');
    ylim([0 Gains.pi_max*1.5]);
    if idx>N3
        xlim([(idx-N3)/10 idx/10]);
    else
        xlim([1 N3/10]);
    end   
    
    subplot(4, 3, 11); % Row 1 spans two columns
    hold on;
    for i = 1:Sims.N
        plot(time(1:idx), Tracker(i).E_Loc(1:idx,1), '-', 'LineWidth', 2, 'Color', AUV_Color(i,:));
    end
    hold off;
    
    xlabel('$t(s)$', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('Errors[m]', 'FontSize', 12, 'Interpreter', 'latex');
    title('Target 1 Estimation Error', 'FontSize', 14, 'Interpreter', 'latex');
    ylim([0 5]);
    if idx>N3
        xlim([(idx-N3)/10 idx/10]);
    else
        xlim([1 N3/10]);
    end   
    
    frame = getframe(fig9); % Capture the figure
    writeVideo(v, frame);
    
    idx=idx+5;
end

close(v);
saveas(fig9, ['figures/' Sims.name '_sim.png']);
savefig(fig9, ['figures/' Sims.name '_sim.fig']); % Save as .fig
