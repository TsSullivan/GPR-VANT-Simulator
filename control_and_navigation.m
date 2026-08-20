function ref = control_and_navigation(t, X, p)
    
    persistent wp_index;
    
    % Resetando inice de navegacao de multiplos destinos
    if isempty(wp_index) || t == 0
        wp_index = 1;
    end
    
    switch p.modo_nav
        
        case 1 % Modo de navegacao para somente um destino
            ref = p.ref; 
            
        case 2 % movimentacao circular
            raio = 5.0;              
            omega = 0.3;                
            centro_x = 10;           
            centro_y = 10;           
            
            x_ref = centro_x + raio * cos(omega * t);
            y_ref = centro_y + raio * sin(omega * t);
            z_ref = 10.0;              
            psi_ref = 0;              
            
            ref = [x_ref; y_ref; z_ref; psi_ref];
            
        case 3 % Navegacao com multiplos destinos
            pos_atual = X(1:3); 
            destino_atual = p.waypoints(1:3, wp_index);
            erro_pos = norm(pos_atual - destino_atual);
            
            tol_pos = 0.15; 
            if erro_pos <= tol_pos && wp_index < size(p.waypoints, 2)
                wp_index = wp_index + 1; 
            end
            
            ref = p.waypoints(:, wp_index);
            
        otherwise
            ref = p.ref; 
            
    end
end