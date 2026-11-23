function plot_graphs(t, X, p, simType)

    if simType == 1
        figure('Name', 'Resultados: Dinâmica básica', 'NumberTitle', 'off', 'Position', [50, 100, 1000, 700]);
        
        % Plot da posicao translacional
        subplot(2,2,1);
        plot(t, X(:,1), t, X(:,2), t, X(:,3), 'LineWidth', 1.5);
        title('Posição translacional'); legend('x', 'y', 'z', 'Location', 'best'); grid on;
        xlabel('Tempo (s)'); ylabel('Posição (m)');
        
        % Angulos de orientacao
        subplot(2,2,2);
        plot(t, X(:,7), t, X(:,8), t, X(:,9), 'LineWidth', 1.5);
        title('Ângulos de orientação'); legend('\phi', '\theta', '\psi', 'Location', 'best'); grid on;
        xlabel('Tempo (s)'); ylabel('Ângulo (rad)');
        
        % Disturbio do vento
        fx_plot = interp1(p.t_wind, p.V_wind_x, t, 'linear', 'extrap');
        fy_plot = interp1(p.t_wind, p.V_wind_y, t, 'linear', 'extrap');
        subplot(2,2,3);
        plot(t, fx_plot, t, fy_plot, 'LineWidth', 1.5);
        title('Turbulência Atmosférica'); legend('v_{wx}', 'v_{wy}', 'Location', 'best'); grid on;
        xlabel('Tempo (s)'); ylabel('Velocidade (m/s)');
        
        % Velocidades lineares
        subplot(2,2,4);
        plot(t, X(:,4), t, X(:,5), t, X(:,6), 'LineWidth', 1.5);
        title('Velocidades Lineares'); legend('v_x', 'v_y', 'v_z', 'Location', 'best'); grid on;
        xlabel('Tempo (s)'); ylabel('Velocidade (m/s)');
        
        drawnow; 
        
    elseif simType == 2
        figure('Name', 'Resultados: Trajetória espacial', 'NumberTitle', 'off', 'Position', [1080, 200, 600, 500]);

        p_traj = plot3(X(:,1), X(:,2), X(:,3), '-b', 'LineWidth', 2); hold on;
        p_init = plot3(X(1,1), X(1,2), X(1,3), 'or', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); 

        if p.modo_nav == 3
            p_wp = plot3(p.waypoints(1, 1:end-1), p.waypoints(2, 1:end-1), p.waypoints(3, 1:end-1), '*k', 'MarkerSize', 10, 'LineWidth', 2);
            p_dest = plot3(p.waypoints(1, end), p.waypoints(2, end), p.waypoints(3, end), '*g', 'MarkerSize', 10, 'LineWidth', 2);
            legend([p_traj, p_init, p_wp, p_dest], 'Trajetória percorrida', 'Ponto Inicial', 'Waypoints', 'Destino', 'Location', 'best');
        else
            p_dest = plot3(p.ref(1), p.ref(2), p.ref(3), '*g', 'MarkerSize', 10, 'LineWidth', 2);
            legend([p_traj, p_init, p_dest], 'Trajetória percorrida', 'Ponto Inicial', 'Destino', 'Location', 'best');
        end
        
        grid on; view(3);
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        title('Trajetória do VANT');
        axis([0 p.map_size 0 p.map_size 0 p.map_size]);
        hold off;
        
        drawnow; 
    elseif simType == 3
        % Dinamica dos rotores e erro de posicao
        figure('Name', 'Resultados: Rotores e Erro', 'NumberTitle', 'off', 'Position', [150, 150, 800, 750]);
        
        subplot(2, 2, 1);
        plot(t, X(:,13), t, X(:,14), t, X(:,15), t, X(:,16), 'LineWidth', 1.5);
        title('Dinâmica dos rotores'); 
        legend('\omega_1', '\omega_2', '\omega_3', '\omega_4', 'Location', 'best'); 
        grid on;
        xlabel('Tempo (s)'); 
        ylabel('Velocidade Angular (rad/s)');
        
        subplot(2, 2, 2);

        N = length(t);
        erro_x = zeros(N, 1);
        erro_y = zeros(N, 1);
        erro_z = zeros(N, 1);
        
        for i = 1:N
            ref_atual = control_and_navigation(t(i), X(i,:)', p); 
            erro_x(i) = ref_atual(1) - X(i,1);
            erro_y(i) = ref_atual(2) - X(i,2);
            erro_z(i) = ref_atual(3) - X(i,3);
        end
        
        plot(t, erro_x, t, erro_y, t, erro_z, 'LineWidth', 1.5);
        hold on;
        yline(0, '--k', 'LineWidth', 1);
        
        title('Erro do rastreamento de posição'); 
        legend('Erro em X', 'Erro em Y', 'Erro em Z', 'Destino', 'Location', 'best'); 
        
        grid on;
        xlabel('Tempo (s)'); 
        ylabel('Erro de Posição (m)');
        hold off;
        
        drawnow; 
    end
end