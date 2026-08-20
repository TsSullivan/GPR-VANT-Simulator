function [value, isterminal, direction] = end_sim(t, X, p)
    % extracao da posicao atual no vetor de estados
    pos = X(1:3);

    % Identifica qual é o alvo com base no modo de navegação
    switch p.modo_nav
        case 1 
            destino = p.ref(1:3);
        case 3 
            destino = p.waypoints(1:3, end);
        otherwise
            % Aqui eh so pra nao encerrar a simulacao em experimentos sem
            % alvos definidos
            destino = [inf; inf; inf]; 
    end

    % Erro de posicao
    erro_pos = norm(pos - destino);
    
    % Raio de detecaco de chegada ao destino
    tol_pos = 0.05; 

    % Calculo para saber se o erro de posicao em realacao ao destino eh
    % zero
    value = erro_pos - tol_pos; 
    
    isterminal = 1; % aqui encerar a simulacao
    direction = -1; 
end