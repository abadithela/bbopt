%% Apurva Badithela
% Compute RSV for mountain car
close all
clear all

%% 5/27/2020
% Computing RSV:
% Load data:
load('cont.mat')
load_system('CM')

%% 
rsv = @(x) (x - 0.6);
delta = 10; % 10s to satisfy robust satisfaction value
N = 100;
RHO = zeros(N*N,1);
X0 = zeros(N,1);
V0 = zeros(N,1);
X = zeros(N*N,1);
V = zeros(N*N,1);
TRAJ = {};
n = 0;
xr = []; rho_r = []; vr = [];
xg = []; rho_g = []; vg = [];
X0 = linspace(-0.66, 0.6, N);
V0 = linspace(-0.42, 0.42, N);
j = 0;
for jx = 1:N
for jv = 1:N
    j = j+1;
    X(j) = X0(jx);
    set_param('CM/CM System/Pos','InitialCondition',num2str(X(j)));
    V(j) = V0(jv);
    set_param('CM/CM System/Vel','InitialCondition',num2str(V(j)));
    simOut = sim('CM','SaveTime','on','TimeSaveName','tout');
    xj = simOut.logsOut.get('pos').Values.Data;
    vj = simOut.logsOut.get('vel').Values.Data;
    tj = simOut.tout;
    st = (simOut.tout(end) < delta);
    if (st)
        rho = max(rsv(xj));
        xg = [xg; X0(jx)];
        vg = [vg; V0(jv)];
        rho_g = [rho_g; rho];
        TRAJ{j} = xj;
    else
        n = n+1;
        tj_less = tj <= delta;
        tj_delta = tj(tj_less == 1);
        n_j = length(tj_delta);
        xj_delta = xj(1:n_j);
        TRAJ{j} = xj_delta;
        rho = max(rsv(xj_delta));
        xr = [xr; X0(jx)];
        vr = [vr; V0(jv)];
        rho_r = [rho_r; rho];
    end
    RHO(j) = rho;
end
end

% No. of violations:
n
%% Plot RSV 
figure(1)
hold on
g = plot(xg, rho_g, '*g', 'MarkerSize',4);
r = plot(xr, rho_r, '*r', 'MarkerSize',4);
xlim([-1.2,0.5])
ylim([-1,1])
xlabel('$X0$','Interpreter','latex')
ylabel('$\rho$','Interpreter','latex')
% legend([g;r], {'Satisfaction of $\dimond_{[0,\delta]}(x>0.6)$','Violation of $\dimond_{[0,\delta]}(x>0.6)$'},'Interpreter','latex');
set(gca,'fontname','times','FontSize',20);
set(gcf, 'PaperUnits', 'inches');
x_width=7.25 ;y_width=7.25;
set(gcf, 'PaperPosition', [0 0 x_width y_width]);

figure(2)
hold on
g1 = plot(vg, rho_g, '*g', 'MarkerSize',4);
r1 = plot(vr, rho_r, '*r', 'MarkerSize',4);
xlim([-1.2,0.5])
ylim([-1,1])
xlabel('$V0$','Interpreter','latex')
ylabel('$rho$','Interpreter','latex')
% legend([g1;r1], {'Satisfaction of $\dimond_{[0,\delta]}(x>0.6)$','Violation of $\dimond_{[0,\delta]}(x>0.6)$'},'Interpreter','latex');
set(gca,'fontname','times','FontSize',20);
set(gcf, 'PaperUnits', 'inches');
x_width=7.25 ;y_width=7.25;
set(gcf, 'PaperPosition', [0 0 x_width y_width]);

figure(3)
hold on
g2 = plot(xg, vg, '*g', 'MarkerSize',4);
r2 = plot(xr, vr, '*r', 'MarkerSize',4);
xlim([-1.2,0.5])
ylim([-1,1])
xlabel('$X0$','Interpreter','latex')
ylabel('$V0$','Interpreter','latex')
% legend([g1;r1], {'Satisfaction of $\dimond_{[0,\delta]}(x>0.6)$','Violation of $\dimond_{[0,\delta]}(x>0.6)$'},'Interpreter','latex');
set(gca,'fontname','times','FontSize',20);
set(gcf, 'PaperUnits', 'inches');
x_width=7.25 ;y_width=7.25;
set(gcf, 'PaperPosition', [0 0 x_width y_width]);

%% 3D line plot
figure(5)
hold on
plot3(xg, vg, rho_g, 'g*');
plot3(xr, vr, rho_r, 'r*');
xlim([-1.2,0.5])
ylim([-1,1])
zlim([-1, 0]);
xlabel('$X0$','Interpreter','latex')
ylabel('$V0$','Interpreter','latex')
zlabel('$rho$','Interpreter','latex')
% legend([g1;r1], {'Satisfaction of $\dimond_{[0,\delta]}(x>0.6)$','Violation of $\dimond_{[0,\delta]}(x>0.6)$'},'Interpreter','latex');
% set(gca,'fontname','times','FontSize',20);
% set(gcf, 'PaperUnits', 'inches');
% x_width=7.25 ;y_width=7.25;
% set(gcf, 'PaperPosition', [0 0 x_width y_width]);

%%
figure(4)
hold on
for j = 1:100
    pathj = TRAJ{j};
    plot([1:length(pathj)]', pathj, 'b--');
end
title('Trajectories of path')

%% Save Data
save(sprintf('X0%d.mat',j),'X0')
save(sprintf('RHO%d.mat',j),'RHO')
save(sprintf('V0%d.mat',j),'V0')

