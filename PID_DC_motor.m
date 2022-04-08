%%%%
clc;close all;clear all;
first_methode = 0;
other_methods = 0;
gain_sensibility = 1;
% parameters
global J b Ke Kt R L
J = 0.02; % 
b = 0.05;
Ke = 0.1;
Kt = 0.2;
R = 3;
L = 0.5;

tfinal = 10;
%sim('PID_DC_MOTOR'); % simulation 

% -------------------------------
% theta_1 = y(:,1);
% theta_2 = y(:,2);
% theta_3 = y(:,3);
% -------------------------------
% err_1 = y(:,4);
% err_2 = y(:,5);
% err_3 = y(:,6);
% -------------------------------
% U_1 = y(:,7);
% U_2 = y(:,8);
% U_3 = y(:,9);
 
 
%  figure 
   
%  subplot (3,2,[1,2])
%  plot(t,theta_dot_1,'b','linewidth',2)
%  plot(t,theta_dot_2,'r','linewidth',2),hold on;
%  plot(t,theta_dot_3,'g','linewidth',2),hold on;
%  xlabel('temps (s)'),ylabel('theta ( rd / s )')
%  legend('theta_dot_1','theta_dot_2','theta_dot_3')
%  grid on,
  
%  subplot (3,2,[3,4])
%  plot(t,err_1,'b','linewidth',2)
%  plot(t,err_2,'r','linewidth',2),hold on;
%  plot(t,err_3,'g','linewidth',2),hold on;
%  xlabel('temps (s)'),ylabel('err( rd / s )')
%  legend('err_1','err_2','err_3')
%  grid on,
  
%  subplot (3,2,[5,6])
%  plot(t,U_1,'b','linewidth',2)
%  plot(t,U_2,'r','linewidth',2),hold on;
%  plot(t,U_3,'g','linewidth',2),hold on;
%  xlabel('temps (s)'),ylabel('U( Volt )')
%  legend('U_1','U_2','U_3')
%  grid on,
%% other methods
if other_methods == 1;
    Kp = 1.58461932315823;
    Ki = 3.79137917089873;
    Kd = 0.154298043345485;
    sim('PID_DC_MOTOR');
    % you can display the scope at the simulation ending by specifying it on
    % configuration properties and mark the case named "Open at simulation
    % start"
    figure(1)
 
    subplot (2,2,1)
    hold on
    f(1) = plot(t,theta_dot_1,'b','linewidth',2);
    f(2) = plot(t,Theta_C,'--k','linewidth',2);
    % create label
    ylabel('$$\dot{\theta}$$','Interpreter','latex','FontSize',14,'FontWeight','bold')
    xlabel('time(s)','FontSize',12,'FontWeight','bold')
    xlim([0 2]);
    % Create legend
    leg = legend([f(1),f(2)],'$$\dot{\theta}$$','$${{\theta}_{Reference}}$$');
    set(leg,'Interpreter','latex','FontSize',14);
    grid on
 
    subplot (2,2,2)
    f(3) = plot(t,err_1,'b','linewidth',2);
    % create label
    ylabel('Error','FontSize',14,'FontWeight','bold')
    xlabel('time(s)','FontSize',12,'FontWeight','bold')

    % Create legend
    leg = legend(f(3),'erreur');
    set(leg,'Interpreter','latex','FontSize',14);
    xlim ([0 2]);
    grid on
 
    subplot (2,2,[3,4])
    f(4) = plot(t,V1,'b','linewidth',2);
    
    % create label
    ylabel('Tension','FontSize',14,'FontWeight','bold')
    xlabel('time(s)','FontSize',12,'FontWeight','bold')
    xlim ([0 2]);
    
    % Create legend
    leg = legend(f(4),'Tension');
    set(leg,'Interpreter','latex','FontSize',14);
    grid on 
end

if gain_sensibility == 1;
    Kp = 1.58461932315823;
    Ki = 3.79137917089873;
    Kd = 0.154298043345485;
    simulation_time = zeros(10,1);
    for k = 1:1:10
        tic
        sim('PID_DC_MOTOR');
        simulation_time(k,1) = toc;
%         hold on
%         plot(k,toc,'*')
%         pause(1)
    end 
    mean(simulation_time);
    % plot figure
    plot(simulation_time,...
        'MarkerFacecolor','green',...
        'MarkerSize',7,...
        'Marker','o',... 
        'LineWidth',2,...
        'Color','blue')
     grid on
   
     for Kp = [0.5 0.8 1 1.6]
         for Ki = [1 4]
             for Kd = [0.05 0.1 0.15]
                 sim('PID_DC_MOTOR');
                 Xm = 5;
                 figure(1)
                 
                 subplot (2,2,1)
                 hold on
                 f(1) = plot(t,theta_dot_1,'linewidth',2);
                 f(2) = plot(t,Theta_C,'--k','linewidth',2);
                 % create label
                 ylabel('$$\dot{\theta}$$','Interpreter','latex','FontSize',14,'FontWeight','bold')
                 xlabel('time(s)','FontSize',12,'FontWeight','bold')
                 xlim([0 Xm]);
                 % Create legend
                 leg = legend([f(1),f(2)],'$$\dot{\theta}$$','$${{\theta}_{Reference}}$$');
                 set(leg,'Interpreter','latex','FontSize',14);
                 grid on
                 
                 subplot (2,2,2)
                 hold on
                 f(3) = plot(t,err_1,'linewidth',2);
                 % create label
                 ylabel('Error','FontSize',14,'FontWeight','bold')
                 xlabel('time(s)','FontSize',12,'FontWeight','bold')
                 
                 % Create legend
                 leg = legend(f(3),'erreur');
                 set(leg,'Interpreter','latex','FontSize',14);
                 xlim ([0 Xm]);
                 grid on
                 
                 subplot (2,2,[3,4])
                 hold on
                 f(4) = plot(t,V1,'linewidth',2);
                 
                 % create label
                 ylabel('Tension','FontSize',14,'FontWeight','bold')
                 xlabel('time(s)','FontSize',12,'FontWeight','bold')
                 xlim ([0 Xm]);
                 
                 % Create legend
                 leg = legend(f(4),'Tension');
                 set(leg,'Interpreter','latex','FontSize',14);
                 grid on
                 pause (1)
             end
         end
         
     end
     
end
    
  