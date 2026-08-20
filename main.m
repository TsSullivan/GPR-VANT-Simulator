% =========================================================================
% Simulador
% =========================================================================
clear; 
close all; 
clc;
disp('Carregando parâmetros do sistema...');
[p, X0, tspan] = init_params();

% Configuracao do tenpo
fps_fisica = 120;          
dt = 1 / fps_fisica;     % Quantos periodos de amostragem tenho em um segundo
fps_tela = 30;            % Atualizacao para a tela
passos_por_quadro = round(fps_fisica / fps_tela); 
t_current = tspan(1);
t_final = tspan(2);
X_current = X0;
contador_loops = 0;

% Variaveis de log
t_log = t_current;
X_log = X_current';

% =========================================================================
% Animacao 3D
% =========================================================================
if p.sim_real_time == 1
    figure('Name', 'Simulador 3D - VANT', 'NumberTitle', 'off', 'Position', [1060, 50, 600, 600]);
    ax = axes('XLim', [0 p.map_size], 'YLim', [0 p.map_size], 'ZLim', [0 p.map_size]);
    view(3); grid on; hold on; 
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Simulação em Tempo Real');
    
    % Plotagem dos destinos
    if p.modo_nav == 3
        plot3(p.waypoints(1, 1:end-1), p.waypoints(2, 1:end-1), p.waypoints(3, 1:end-1), '*k', 'MarkerSize', 10, 'LineWidth', 2);
        plot3(p.waypoints(1, end), p.waypoints(2, end), p.waypoints(3, end), '*g', 'MarkerSize', 10, 'LineWidth', 2);
    else
        plot3(p.ref(1), p.ref(2), p.ref(3), '*g', 'MarkerSize', 10, 'LineWidth', 2);
    end
    
    % Estrutura visual inicial do drone
    L = p.l; 
    arm1_base = [L, -L; 0, 0; 0, 0]; 
    arm2_base = [0, 0; L, -L; 0, 0];
    
    pos_ini = [X_current(1); X_current(2); X_current(3)]; 
    arm1_ini = arm1_base + pos_ini;
    arm2_ini = arm2_base + pos_ini;
    
    arm1_plot = plot3(ax, arm1_ini(1,:), arm1_ini(2,:), arm1_ini(3,:), '-r', 'LineWidth', 3);
    arm2_plot = plot3(ax, arm2_ini(1,:), arm2_ini(2,:), arm2_ini(3,:), '-b', 'LineWidth', 3);
    
    disp('A simulação vai começar em 3 segundos...');
    pause(3);
else
    disp('Modo de simulação rápida (instantânea) ativado. Calculando...');
end

% =========================================================================
% Loop de chamada do Runge Kutta de quarta ordem
% =========================================================================
if p.sim_real_time == 1
    disp('Iniciando a simulação em malha fechada sincronizada...');
end

% Temporizador da simulacao
sim_start_time = tic; 

while t_current < t_final
    % RK4 
    k1 = init_dynamics(t_current, X_current, p);
    k2 = init_dynamics(t_current + dt/2, X_current + (dt/2)*k1, p);
    k3 = init_dynamics(t_current + dt/2, X_current + (dt/2)*k2, p);
    k4 = init_dynamics(t_current + dt, X_current + dt*k3, p);
    
    X_current = X_current + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
    t_current = t_current + dt;
    
    t_log = [t_log; t_current];
    X_log = [X_log; X_current'];
    
    % 2. Atualização da Tela (Executada apenas se a animação estiver ativa)
    if p.sim_real_time == 1 && mod(contador_loops, passos_por_quadro) == 0
        pos = [X_current(1); X_current(2); X_current(3)];
        phi = X_current(7); theta = X_current(8); psi = X_current(9);
        
        Cphi = cos(phi); Sphi = sin(phi); 
        Ctheta = cos(theta); Stheta = sin(theta); 
        Cpsi = cos(psi); Spsi = sin(psi);
        
        R = [Cpsi*Ctheta, Cpsi*Stheta*Sphi - Spsi*Cphi, Cpsi*Stheta*Cphi + Spsi*Sphi;
             Spsi*Ctheta, Spsi*Stheta*Sphi + Cpsi*Cphi, Spsi*Stheta*Cphi - Cpsi*Sphi;
             -Stheta,     Ctheta*Sphi,                  Ctheta*Cphi];
         
        arm1_rot = R * arm1_base + pos; 
        arm2_rot = R * arm2_base + pos;
        
        set(arm1_plot, 'XData', arm1_rot(1,:), 'YData', arm1_rot(2,:), 'ZData', arm1_rot(3,:));
        set(arm2_plot, 'XData', arm2_rot(1,:), 'YData', arm2_rot(2,:), 'ZData', arm2_rot(3,:));
        
        drawnow limitrate; 
    end
    
    % Verificacao de chegada ao destino
    [evento_valor, ~, ~] = end_sim(t_current, X_current, p);
    if evento_valor <= 0 
        disp('Destino final atingido pelo evento de tolerância!');
        break;
    end
    
    % Sincronizacao da somulacao
    if p.sim_real_time == 1
        tempo_decorrido_real = toc(sim_start_time);
        if tempo_decorrido_real < t_current
            % Pequeno atraso para parear o clock do simulador com o da
            % maquina
            pause(t_current - tempo_decorrido_real);
        end
    end
    
    contador_loops = contador_loops + 1;
end

tempo_decorrido_total = toc(sim_start_time);

if p.sim_real_time == 1
    fprintf('Voo concluído! Tempo real de execução: %.2f segundos.\n', tempo_decorrido_total);
else
    fprintf('Cálculo numérico finalizado em %.2f segundos.\n', tempo_decorrido_total);
end

% =========================================================================
% Graficos
% =========================================================================
disp('Gerando relatórios e gráficos de análise...');
plot_graphs(t_log, X_log, p, 1); 
plot_graphs(t_log, X_log, p, 2); 
plot_graphs(t_log, X_log, p, 3); 
disp('Simulador encerrado.');