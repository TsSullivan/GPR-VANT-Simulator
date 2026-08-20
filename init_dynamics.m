function dX = init_dynamics(t, X, p)
    % Extracao dos estados gerais do sistema
    x = X(1); y = X(2); z = X(3);
    dx = X(4); dy = X(5); dz = X(6);
    phi = X(7); theta = X(8); psi = X(9);
    dphi = X(10); dtheta = X(11); dpsi = X(12);
    w_real = X(13:16); % leituras das velocidades angulares
    int_e_z = X(17); % integral do erro em Z
    int_e_x = X(18); % integral do erro em X
    int_e_y = X(19); % integral do erro em Y
    
    % Requisicao do sistema de navegacao
    ref_atual = control_and_navigation(t, X, p);
    
    % Extracao das referencias nas coordenadas
    x_ref = ref_atual(1); 
    y_ref = ref_atual(2); 
    z_ref = ref_atual(3); 
    psi_ref = ref_atual(4);
    
    % =========================================================
    % CONTROLE DE ALTITUDE (EIXO Z)
    % =========================================================
    limite_erro = 1.5; % Limitindo o erro
    e_z = max(-limite_erro, min(limite_erro, z_ref - z)); 
    e_dz = 0 - dz;
    
    termo_integral = p.Ki_z * int_e_z;
    limite_integral = 0.5; 
    
    if abs(e_z) < 1.0
        d_int_e_z = e_z; 
    else
        d_int_e_z = 0;
    end
    
    if (termo_integral >= limite_integral && e_z > 0) || (termo_integral <= -limite_integral && e_z < 0)
        d_int_e_z = 0;
    end
    
    f_T_cmd = max(0, p.m * p.g + p.Kp_z * e_z + p.Kd_z * e_dz + termo_integral);

    % =========================================================
    % CONTROLE TRANSLACIONAL 
    % =========================================================
    limite_integral_xy = 2.0; 
    limite_aceleracao = 0.35 * p.g; 
    v_max = 2.5; % V_max em m/s
    
    % Erro do eixo x
    e_x = max(-limite_erro, min(limite_erro, x_ref - x)); 
    
    % Malha de posicao
    v_ref_x = p.Kp_x * e_x;
    v_ref_x = max(-v_max, min(v_max, v_ref_x)); 
    
    % Malha de velocidade com erro relativo
    e_vx = v_ref_x - dx;
    a_x_pd = p.Kd_x * e_vx;
    
    % Anti-windup
    if (a_x_pd >= limite_aceleracao && e_vx > 0) || (a_x_pd <= -limite_aceleracao && e_vx < 0)
        d_int_e_x = 0;
    else
        d_int_e_x = e_vx; 
    end
    
    termo_integral_x = p.Ki_x * int_e_x;
    
    if (termo_integral_x >= limite_integral_xy && e_vx > 0) || (termo_integral_x <= -limite_integral_xy && e_vx < 0)
        d_int_e_x = 0;
    end
    
    a_x_cmd = a_x_pd + termo_integral_x;
    theta_ref = max(-0.35, min(0.35, a_x_cmd / p.g)); 
    
    % Eixo Y
    e_y = max(-limite_erro, min(limite_erro, y_ref - y));
    
    v_ref_y = p.Kp_y * e_y;
    v_ref_y = max(-v_max, min(v_max, v_ref_y));
    
    e_vy = v_ref_y - dy;
    a_y_pd = p.Kd_y * e_vy;
    
    if (a_y_pd >= limite_aceleracao && e_vy > 0) || (a_y_pd <= -limite_aceleracao && e_vy < 0)
        d_int_e_y = 0;
    else
        d_int_e_y = e_vy;
    end
    
    termo_integral_y = p.Ki_y * int_e_y;
    
    if (termo_integral_y >= limite_integral_xy && e_vy > 0) || (termo_integral_y <= -limite_integral_xy && e_vy < 0)
        d_int_e_y = 0;
    end
    
    a_y_cmd = a_y_pd + termo_integral_y;
    phi_ref = max(-0.35, min(0.35, -a_y_cmd / p.g));
    
    % =========================================================
    % CONTROLE DE ATITUDE (MALHA INTERNA)
    % =========================================================
    tau_phi_cmd   = p.Kp_phi * (phi_ref - phi)     + p.Kd_phi * (0 - dphi);
    tau_theta_cmd = p.Kp_theta * (theta_ref - theta) + p.Kd_theta * (0 - dtheta);
    tau_psi_cmd   = p.Kp_psi * (psi_ref - psi)     + p.Kd_psi * (0 - dpsi);
    
    % Alocacao individual dos motores
    U = [f_T_cmd; tau_phi_cmd; tau_theta_cmd; tau_psi_cmd];
    w_sq = p.M_inv * U; 
    
    % Definicao do limite minimo de rotacao dos motores em rad/s
    w_idle = 150;
    
    w_cmd = min(p.w_max, max(w_idle, sqrt(max(0, w_sq)))); 
    
    % =========================================================
    % Dinamica
    % =========================================================
    Cphi = cos(phi); Sphi = sin(phi); Ctheta = cos(theta); Stheta = sin(theta); Cpsi = cos(psi); Spsi = sin(psi);
    w1 = w_real(1); w2 = w_real(2); w3 = w_real(3); w4 = w_real(4);
    
    % Efeito solo
    z_solo = max(z, 0.02); 
    if z_solo < (2 * p.R_rotor)
        fator_solo = 1 / (1 - (p.R_rotor / (4 * z_solo))^2);
        fator_solo = max(1.0, min(fator_solo, 2.0)); 
    else
        fator_solo = 1.0; 
    end
    
    f_T = fator_solo * p.ke * sum(w_real.^2);

    tau_B = [p.l * p.ke * (w2^2 - w4^2);
             p.l * p.ke * (w1^2 - w3^2);
             p.km * (-w1^2 + w2^2 - w3^2 + w4^2)];
             
    dw_real = (w_cmd - w_real) / p.tau_m;
    
    R13 = Cpsi * Stheta * Cphi + Spsi * Sphi;
    R23 = Spsi * Stheta * Cphi - Cpsi * Sphi;
    R33 = Ctheta * Cphi;
    
    % Vento e Arrasto aerodinamico (MOdelo de Dryden)
    vw_x = interp1(p.t_wind, p.V_wind_x, t, 'linear', 'extrap');
    vw_y = interp1(p.t_wind, p.V_wind_y, t, 'linear', 'extrap');
    
    v_rel_x = dx - vw_x;
    v_rel_y = dy - vw_y;
    v_rel_z = dz - 0; 
    
    ddx = -(p.Ax/p.m)*v_rel_x + (f_T/p.m) * R13;
    ddy = -(p.Ay/p.m)*v_rel_y + (f_T/p.m) * R23;
    ddz = -p.g - (p.Az/p.m)*v_rel_z + (f_T/p.m) * R33;
    
    % Matriz de inercia e coriolis
    J_eta = [p.Ixx, 0, -p.Ixx * Stheta;
             0, p.Iyy * Cphi^2 + p.Ixx * Sphi^2, (p.Iyy - p.Izz) * Cphi * Sphi * Ctheta;
             -p.Ixx * Stheta, (p.Iyy - p.Izz) * Cphi * Sphi * Ctheta, p.Ixx * Stheta^2 + p.Iyy * Sphi^2 * Ctheta^2 + p.Izz * Ctheta^2];
             
    C_eta = [0, (p.Iyy - p.Izz)*(dtheta*Cphi*Sphi + dpsi*Sphi^2*Ctheta) + (p.Izz - p.Iyy)*dpsi*Cphi^2*Ctheta - p.Ixx*dpsi*Ctheta, (p.Izz - p.Iyy)*dpsi*Cphi*Sphi*Ctheta^2;
             (p.Izz - p.Iyy)*(dtheta*Cphi*Sphi + dpsi*Sphi*Ctheta) + (p.Iyy - p.Izz)*dpsi*Cphi^2*Ctheta + p.Ixx*dpsi*Ctheta, (p.Izz - p.Iyy)*dphi*Cphi*Sphi, -p.Ixx*dpsi*Stheta*Ctheta + p.Iyy*dpsi*Sphi^2*Stheta*Ctheta + p.Izz*dpsi*Cphi^2*Stheta*Ctheta;
             (p.Iyy - p.Izz)*dpsi*Ctheta^2*Sphi*Cphi - p.Ixx*dtheta*Ctheta, (p.Izz - p.Iyy)*(dtheta*Cphi*Sphi*Stheta + dphi*Sphi^2*Ctheta) + (p.Iyy - p.Izz)*dphi*Cphi^2*Ctheta + p.Ixx*dpsi*Stheta*Ctheta - p.Iyy*dpsi*Sphi^2*Stheta*Ctheta - p.Izz*dpsi*Cphi^2*Stheta*Ctheta, (p.Iyy - p.Izz)*dphi*Cphi*Sphi*Ctheta^2 - p.Iyy*dtheta*Sphi^2*Ctheta*Stheta - p.Izz*dtheta*Cphi^2*Ctheta*Stheta + p.Ixx*dtheta*Ctheta*Stheta];
             
    eta_ddot = J_eta \ (tau_B - C_eta * [dphi; dtheta; dpsi]);
    
    % Saida do vetor de estados
    dX = [dx; dy; dz; ddx; ddy; ddz; dphi; dtheta; dpsi; eta_ddot(1); eta_ddot(2); eta_ddot(3); dw_real; d_int_e_z; d_int_e_x; d_int_e_y];
end