function [p, X0, tspan] = init_params()
    % Configuracao da simulacao
    % 0 -> Simulacao "instantanea"
    % 1 -> Animacao 3D em tempo real
    p.sim_real_time = 1;
    p.map_size = 20;

    % Parametros do drone (DJI mini Mavic)
    p.g = 9.81; % Aceleracao da gravidade (m/s^2)                
    p.m = 0.249; % Massa do drone (kg)                
    p.l = 0.1235; % Distancia do centro de massa do drone ate um motor (m)  
    p.R_rotor = 0.06; % Raio da helice do rotor (m)
    p.ke = 1.45e-6;              
    p.km = 2.30e-8;              
    p.IM = 1.5e-5;               
    p.Ixx = 0.85e-3;             
    p.Iyy = 1.05e-3;             
    p.Izz = 1.70e-3;             
    p.Ax = 0.15;                 
    p.Ay = 0.15;                 
    p.Az = 0.20;                 
    p.tau_m = 0.05;   
    p.w_max = 1500; % Limite de rotacao dos motores em rad/s

    % Tempo de simulacao 
    tspan = [0 50]; % em segundos

    % modelo de Dryden
    rng('shuffle'); 
    
    p.t_wind = linspace(tspan(1), tspan(2), 2000); 
    dt = p.t_wind(2) - p.t_wind(1); 
    
    % Parametos que quero para o vento
    L_u = 10.0;      % Complimento de turbulencia (m)
    sigma_u = 1.06;  % Intensidade (m/s)
    V_nominal = 1.0; % Velocidade (m/s)
    
    % Funcao de transferencia longitudinal de Dryden
    numerador = sigma_u * sqrt((2 * L_u) / (pi * V_nominal));
    denominador = [(L_u / V_nominal), 1];
    filtro_dryden = tf(numerador, denominador);
    
    % Ruido branco e filtro
    ruido_branco_x = randn(1, length(p.t_wind)) / sqrt(dt);
    rajadas_x = lsim(filtro_dryden, ruido_branco_x, p.t_wind)';
    
    % Sorteio de vento para eixo X
    vento_base_x = 0.5 + (2.0 * rand()); 
    p.V_wind_x = vento_base_x + rajadas_x; 
    
    % No eixo Y
    ruido_branco_y = randn(1, length(p.t_wind)) / sqrt(dt);
    rajadas_y = lsim(filtro_dryden, ruido_branco_y, p.t_wind)';
    vento_base_y = -1.0 + (2.0 * rand());
    p.V_wind_y = vento_base_y + (0.5 * rajadas_y); 


    % Posicao inicial do drone 
    X0 = zeros(19, 1);
    X0(1) = 1; % X
    X0(2) = 1; % Y
    X0(3) = 1; % Z

    % Variavel que define testes de modos de navegacao
    % 1 = Somente um destino, 2 = Circular, 3 = Multiplos destinos
    p.modo_nav = 3;
    
    % Destino no modo 1 [x, y, z, psi]
    p.ref = [4.5; 3; 4.5; 0]; 
    
    % Matriz que contem as coordenadas dos destinos do modo 3
    p.waypoints = [2,  1,  3,  15;   % Coordenadas X
                   4,  15,  9,  4;   % Coordenadas Y
                   7,  7, 0.03,  0.03;   % Coordenadas Z
                   0,  0,  0,  0];  % Angulo Psi

    % Ganhos do controlador PID 
    p.Kp_z = 2.0; p.Kd_z = 2.0; p.Ki_z = 1.5;
    p.Kp_x = 0.8; p.Kd_x = 1.5; p.Ki_x = 1.5;
    p.Kp_y = 0.8; p.Kd_y = 1.5; p.Ki_y = 1.5;
    
    % Malha interna da aeronave
    p.Kp_phi = 0.02;   p.Kd_phi = 0.005;
    p.Kp_theta = 0.02; p.Kd_theta = 0.005;
    p.Kp_psi = 0.02;   p.Kd_psi = 0.005;

    % Matriz mixer
    p.M_mix = [p.ke,       p.ke,       p.ke,       p.ke;
               0,          p.l*p.ke,   0,         -p.l*p.ke;
               p.l*p.ke,   0,         -p.l*p.ke,   0;
              -p.km,       p.km,      -p.km,       p.km];
    p.M_inv = inv(p.M_mix);
end