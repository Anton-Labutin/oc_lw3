%% Практикум по ОУ

% Лабораторная работа №3
% Построение множества достижимости
% Вариант 5

clear;
clc;

%% входные данные
global t_0;
t_0 = 0;

global x0;
x0 = [0; 0];

T = 1.0;

alpha = 1.0;

t1 = 0.3;
t2 = 0.8;

N = 20;

%%

mov = reachsetdyn(alpha, t1, t2, N, "my_video");

figure('Position', [100 100 1000 1000]);
hold on;

[X, Y, u1_switch, u2_switch] = reachset(0.25, T);

plot(X, Y, 'k', 'LineWidth', 1);
plot(u1_switch(1, :), u1_switch(2, :), 'ro', 'MarkerSize', 3);
plot(u2_switch(1, :), u2_switch(2, :), 'go', 'MarkerSize', 3);
plot([X, X(1)], [Y, Y(1)], 'k');

[X, Y, u1_switch, u2_switch] = reachset(0.75, T);

plot(X, Y, 'k', 'LineWidth', 1);
plot(u1_switch(1, :), u1_switch(2, :), 'ro', 'MarkerSize', 3);
plot(u2_switch(1, :), u2_switch(2, :), 'go', 'MarkerSize', 3);
plot([X, X(1)], [Y, Y(1)], 'k');

[X, Y, u1_switch, u2_switch] = reachset(1.25, T);

plot(X, Y, 'k', 'LineWidth', 1);
plot(u1_switch(1, :), u1_switch(2, :), 'ro', 'MarkerSize', 3);
plot(u2_switch(1, :), u2_switch(2, :), 'go', 'MarkerSize', 3);
plot([X, X(1)], [Y, Y(1)], 'k');

[X, Y, u1_switch, u2_switch] = reachset(1.75, T);

plot(X, Y, 'k', 'LineWidth', 1);
plot(u1_switch(1, :), u1_switch(2, :), 'ro', 'MarkerSize', 3);
plot(u2_switch(1, :), u2_switch(2, :), 'go', 'MarkerSize', 3);
plot([X, X(1)], [Y, Y(1)], 'k');

[X, Y, u1_switch, u2_switch] = reachset(2.0, T);

plot(X, Y, 'k', 'LineWidth', 1);
plot(u1_switch(1, :), u1_switch(2, :), 'ro', 'MarkerSize', 3);
plot(u2_switch(1, :), u2_switch(2, :), 'go', 'MarkerSize', 3);
plot([X, X(1)], [Y, Y(1)], 'k');

hold off;
xlabel('x_1');
ylabel('x_2');



function [X_res, Y_res, u1_switch, u2_switch] = reachset(alpha, T)
    global t_0;
    global x0;
    
    if alpha <= 0
        error('reachset: alpha > 0');
    end
    
    if T < t_0
        error('reachset : t >= t_0');
    end
    
    eps = 1e-3;
    
    options1 = odeset('Events', @x_2_event_fun, 'RelTol', eps, 'AbsTol', eps);
    options2 = odeset('Events', @psi_2_event_fun, 'RelTol', eps, 'AbsTol', eps);
    
    X=[];
    Y=[];
    
    time_limits=[t_0, T];
    v0 = x0;
    
    u_opt = [alpha, -alpha];
    
    for u_idx = 1 : size(u_opt, 2)
        u = u_opt(1, u_idx);
        
        [t, y, te, ye, ie] = ode45(@(t, x) subsystem(t, x, u), time_limits, v0, options1);
        
        X_new = [X, y(:, 1)'];
        Y_new = [Y, y(:, 2)'];
        clear X Y;
        X = X_new;
        Y = Y_new;
        clear X_new Y_new;
        
        if u_idx == 1
            u1_switch = zeros(2, size(y, 1));
            u1_switch(1, :) = y(:, 1)';
            u1_switch(2, :) = y(:, 2)';
        else 
            u2_switch = zeros(2, size(y, 1));
            u2_switch(1, :) = y(:, 1)';
            u2_switch(2, :) = y(:, 2)';
        end
        
        for i = 2 : size(t)
            t_i = t(i);
            v0_i = [y(i, 1); y(i, 2); 1; 0];
            next_u = -u;
        
            while(t_i < T)
                [t_2, y_2, te_i, ye_i, ie_i] = ... 
                    ode45(@(t, x) system(t, x, next_u), time_limits, v0_i, options2);
             
                plot(y_2(:, 1), y_2(:, 2), 'y');
                
                X_new = [X, y_2(:, 1)'];
                Y_new = [Y, y_2(:, 2)'];
                clear X Y
                X = X_new;
                Y = Y_new;
                clear X_new Y_new;
                
                t_i = t_2(end);
                v0_i = [y_2(end, 1); y_2(end, 2); -1 * sign(next_u); 0];
                next_u = -next_u;
                clear t_2 y_2
            end
        end
        
        clear t y;
    end
    
    border_indices = boundary(X', Y');
    X_res = X(border_indices);
    Y_res = Y(border_indices);
end


function [mov] = reachsetdyn(alpha, t1, t2, N, filename)
    mov(1 : N) = struct('cdata', [], 'colormap', []);
    time_step = (t2 - t1) / N;
     
    figure('Position', [100 100 1000 1000]);
    hold on;
    
    for i = 1 : N
        [X, Y, u1_switch, u2_switch] = reachset(alpha, t1 + i * time_step);
        
        plot(X, Y, 'k', 'LineWidth', 1);
        plot(u1_switch(1, :), u1_switch(2, :), 'ro', 'MarkerSize', 2);
        plot(u2_switch(1, :), u2_switch(2, :), 'go', 'MarkerSize', 2);
        
        mov(i) = getframe();
    end
    hold off;
    xlabel('x_1');
    ylabel('x_2');
    
    if nargin == 5 & filename ~= ""
        video = VideoWriter(filename);
        video.FrameRate = 1;
        open(video);
        writeVideo(video, mov);
        close(video);
    end
end


function [value, isterminal, direction] = x_2_event_fun(t, x) % Event
    value(1) = x(2);
    isterminal(1) = 1;
    direction(1) = 0;
end


function [value, isterminal, direction] = psi_2_event_fun(t, x) % Event2
    value(1) = x(4);
    isterminal(1) = 1;
    direction(1) = 0;
end


function [sol] = system(t, x, u)
    % d(x_1) / dt = x_2, 
    % d(x_2) / dt = (x_1)^2 - x_2 * sin(2 * (x_1)^2) + u
    % d(psi_1) / dt = -2 * psi_2 * x_1 + 4 * x_1 * x_2 * psi_2 * cos(2 * (x_1)^2)
    % d(psi_2) / dt = -psi_1 + psi_2 * sin(2 * (x_1)^2)
    
    sol = ones(4, 1);
    
    sol(1, 1) = x(2, 1);
    
    sol(2, 1) = (x(1, 1))^2 - x(2, 1) * sin(2 * (x(1, 1))^2) + u;
    
    sol(3, 1) = -2 * x(4, 1) * x(1, 1) + ...
        4 * x(1, 1) * x(2, 1) * x(4, 1) * cos(2 * (x(1, 1))^2);
    
    sol(4, 1) = -x(3, 1) + x(4, 1) * sin(2 * (x(1, 1))^2);
end


function[sol] = subsystem(t, x, u) % diff
    % dx_1 / dt = x_2, 
    % dx_2 / dt = (x_1)^2 - x_2 * sin(2 * (x_1)^2) + u
    
    sol = ones(2, 1);
    
    sol(1, 1) = x(2, 1);
    
    sol(2, 1) = (x(1, 1))^2 - x(2, 1) * sin(2 * (x(1, 1))^2) + u;
end