% EJECUTADOR
clearvars -except i_exe folder
close all


% Inicialización del mapa y trayectoria

global secdist free_col;
secdist = 0.7;
free_col = 1;
t = 1;
tolerance = 0.075;
target_reached = 0;

robot = 'Marvin';
laser = 'LMS200';

if exist('i_exe', 'var')
    c = i_exe(1);
else
    c = 22;
end

experiment = '1';

speedk = 0.1;
speed_control = readfis('SpeedController.fis');

load(strcat('Experimentos/trayectoria', experiment));
[wide, long, obstacles, balizas, puertas] = initWorld(world);


%----------------------------------
% VARIABLES COMUNES DE LOS FILTROS:
%----------------------------------

% Cargamos el bias b, y las matrices de incertidumbre Q, R, y B:
calib = strcat('calibraciones/calibracion', num2str(c), '.mat');
load(calib);

% Inicialización de P(0|0):
Px = 0.001;
Py = 0.001;
Pth = 0.001;
Pk = [Px 0 0; 0 Py 0; 0 0 Pth];

% Posicionamos el robot en la posición inicial Xk(0|0):
xini = start(1);
yini = start(2);
thini = pi/4;

apoloUpdate(world);
apoloPlaceMRobot(robot, [xini yini 0], thini, world);
apoloResetOdometry(robot, [0 0 0], world);
apoloGetLaserData(laser, world);
apoloUpdate(world);

% Registramos valores iniciales:
aux = apoloGetLocationMRobot(robot, world);
Xkreal = [aux(1); aux(2); aux(4)];
Xk = Xkreal;
Euk_local = b(1:3);

Xreal(:,t) = Xkreal;
t_i = t;
fig_id = 1;
speed(t) = speedk;


%----------------------------------
% VARIABLES PROPIAS DE CADA FILTRO:
%----------------------------------
%------------------------------------------------------------
% EKF
Xk_Ekf = Xk;
Pk_Ekf = Pk;
Xestimado_Ekf(:,t) = Xk_Ekf;
Pestimado_Ekf(:,t) = [Pk_Ekf(1,1); Pk_Ekf(2,2); Pk_Ekf(3,3)];

    % Tiempo de cada iteración:
    time_Ekf(t) = 0;
%------------------------------------------------------------

%------------------------------------------------------------
% ESKF
Xk_Eskf = Xk;
Pk_Eskf = Pk;
Xestimado_Eskf(:,t) = Xk_Eskf;
Pestimado_Eskf(:,t) = [Pk_Eskf(1,1); Pk_Eskf(2,2); Pk_Eskf(3,3)];

    % Vector bias b y matriz Bk:    
%     b_Eskf = b(4:5);
%     Bk_Eskf = Bk(4:5,4:5);
    b_Eskf = b;
    Bk_Eskf = Bk;
    nb_Eskf = length(b_Eskf);

    % Inicialización de Pxz(0|0):
    Pxbk_Eskf = zeros(3,nb_Eskf);
    
    % Tiempo de cada iteración:
    time_Eskf(t) = 0;
%------------------------------------------------------------

%------------------------------------------------------------
% UKF
Xk_Ukf = Xk;
Pk_Ukf = Pk;
Xestimado_Ukf(:,t) = Xk_Ukf;
Pestimado_Ukf(:,t) = [Pk_Ukf(1,1); Pk_Ukf(2,2); Pk_Ukf(3,3)];

    % Variables del filtro:
    q_Ukf = size(Q,1);
    sv_aug_Ukf = q_Ukf + 3;
    sv_aug_sigma_Ukf = 2*sv_aug_Ukf + 1;
    alpha_Ukf = 0.001;                               
    beta_Ukf = 2;                          
    kappa_Ukf = 0;
    lambda_Ukf = alpha_Ukf^2 * (sv_aug_Ukf + kappa_Ukf) - sv_aug_Ukf;
    
    % Pesos:
    wm_Ukf = zeros(1, sv_aug_sigma_Ukf);
    wc_Ukf = zeros(1, sv_aug_sigma_Ukf);

    wm_Ukf(1) = lambda_Ukf / (sv_aug_Ukf + lambda_Ukf);
    wc_Ukf(1) = lambda_Ukf / (sv_aug_Ukf + lambda_Ukf) + (1 - alpha_Ukf^2 + beta_Ukf);

    for i = 2:sv_aug_sigma_Ukf
        wm_Ukf(i) = 1/(2*(sv_aug_Ukf + lambda_Ukf));
        wc_Ukf(i) = 1/(2*(sv_aug_Ukf + lambda_Ukf));
    end
    
    % Tiempo de cada iteración:
    time_Ukf(t) = 0;

%------------------------------------------------------------

%------------------------------------------------------------
% USKF
Xk_Uskf = Xk;
Pk_Uskf = Pk;
Xestimado_Uskf(:,t) = Xk_Uskf;
Pestimado_Uskf(:,t) = [Pk_Uskf(1,1); Pk_Uskf(2,2); Pk_Uskf(3,3)];   

    % Vector bias b y matriz Bk:
%     b_Uskf = b(4:5);
%     Bk_Uskf = Bk(4:5,4:5);
    b_Uskf = b;
    Bk_Uskf = Bk;
    nb_Uskf = length(b_Uskf);

    % Inicialización de Pxz(0|0):
    Pxbk_Uskf = zeros(3,nb_Uskf);
    
    XkAug_Uskf = [Xk_Uskf; b_Uskf];
    PkAug_Uskf = [Pk_Uskf Pxbk_Uskf; Pxbk_Uskf' Bk_Uskf]; 
    
    % Variables del filtro:
    nx_Uskf = 3;
    sv_aug_Uskf = nx_Uskf + nb_Uskf;
    sv_aug_sigma_Uskf = 2*sv_aug_Uskf + 1;
    alpha_Uskf = 0.001;
    beta_Uskf = 2;
    kappa_Uskf = 0;
    lambda_Uskf = alpha_Uskf^2 * (sv_aug_Uskf + kappa_Uskf) - sv_aug_Uskf;

    wm_Uskf = zeros(1, sv_aug_sigma_Uskf);
    wc_Uskf = zeros(1, sv_aug_sigma_Uskf);

    wm_Uskf(1) = lambda_Uskf / (sv_aug_Uskf + lambda_Uskf);
    wc_Uskf(1) = lambda_Uskf / (sv_aug_Uskf + lambda_Uskf) + (1 - alpha_Uskf^2 + beta_Uskf);

    for i = 2:sv_aug_sigma_Uskf
        wm_Uskf(i) = 1/(2*(sv_aug_Uskf + lambda_Uskf));
        wc_Uskf(i) = 1/(2*(sv_aug_Uskf + lambda_Uskf));
    end
    
    % Tiempo de cada iteración:
    time_Uskf(t) = 0;
    
%----------------------------------
% MOVIMIENTO:
%----------------------------------
while target_reached == 0 && free_col
    apoloResetOdometry(robot, [0 0 0], world);
    t = t + 1;
    
    distanceError = sqrt((path(1,1) - Xkreal(1))^2 + (path(1,2)-Xkreal(2))^2);
    angleError = fixAngleRad(atan2(path(1,2) - Xkreal(2), path(1,1) - Xkreal(1)) - Xkreal(3));
    speedk = evalfis(speed_control, [distanceError angleError])/1000;
    speed(t) = speedk;
    
    secuence = [orientate2Target(Xkreal(1:2)', path(1,:), Xkreal(3)); 
                0.5 0 speedk];

    for i = 1:size(secuence,1)
        free_col = apoloMoveMRobot(robot, secuence(i,1:2), secuence(i,3), world);
        apoloUpdate(world);
%         pause(0.005)
    end
    
    
 % LOCALIZACION:
    
    aux = apoloGetLocationMRobot(robot, world);
    Xkreal = [aux(1) aux(2) aux(4)]';
    Xreal(:,t) = Xkreal;

    Xk_1 = Xk;
    Pk_1 = Pk;
    
      % RUIDO
        if     c == 12 
            noise_param = [0, 0;                      0, 0;                       0, 0;                       0.2 + 0.1 * cos(t), 0;      0.2 + 0.1 * cos(t), 0];
        elseif c == 22
            noise_param = [0.2 + 0.1 * cos(t), 0;     0.2 + 0.1 * cos(t), 0;      0.2 + 0.1 * cos(t), 0;      0, 0;                       0, 0                 ];
        elseif c == 32 
            noise_param = [0.2 + 0.1 * cos(t), 0;     0.2 + 0.1 * cos(t), 0;      0.2 + 0.1 * cos(t), 0;      0.2 + 0.1 * cos(t), 0;      0.2 + 0.1 * cos(t), 0];
        end
        

    % OBSERVACIÓN:
    aux = apoloGetLaserLandMarks(laser, world);
    
        % Calculamos las medidas sin corregir (biased) y corregidas (unbiased):
        Zk_biased = [];
        for i=1:length(aux.id)
            Zk_biased = [Zk_biased;
                         aux.distance(i) + normalNoise(noise_param(4,1), noise_param(4,2));
                         fixAngleRad(aux.angle(i) + normalNoise(noise_param(5,1), noise_param(5,2)))];
        end

        Zk_unbiased = Zk_biased;
        for i=1:length(aux.id)
            Zk_unbiased(2*i-1) = Zk_biased(2*i-1) + b(4);
            Zk_unbiased(2*i) = fixAngleRad(Zk_biased(2*i) + b(5));
        end
    
    Zk = Zk_biased;
    
    
    % ODOMETRIA
    % Añadimos un ruido de distribución normal a la odometría:
    noisek = [normalNoise(noise_param(1,1), noise_param(1,2));
             normalNoise(noise_param(2,1), noise_param(2,2)); 
             normalNoise(noise_param(3,1), noise_param(3,2))];
             
    apoloGetOdometry(robot, world)'
    uk = apoloGetOdometry(robot, world)'  + noisek;
    uk(3,1) = fixAngleRad(uk(3,1));
    
    
    % FILTROS
    tic
    [Xk_Ekf, Pk_Ekf] = EkfTask(balizas, Q, R_angle, R_distance, uk, Euk_local, Xk_Ekf, Pk_Ekf, Zk, aux);
    timek = toc;
    
        % REGISTRO:
        Xestimado_Ekf(:,t) = Xk_Ekf;
        Pestimado_Ekf(:,t) = [Pk_Ekf(1,1); Pk_Ekf(2,2); Pk_Ekf(3,3)];
        time_Ekf(t) = timek;
    
        
    tic    
    [Xk_Eskf, Pk_Eskf, Pxbk_Eskf] = EskfTask(balizas, Q, R_angle, R_distance, b_Eskf, Bk_Eskf, uk, Euk_local, nb_Eskf, Xk_Eskf, Pk_Eskf, Pxbk_Eskf, Zk, aux);
    timek = toc;
    
        % REGISTRO:
        Xestimado_Eskf(:,t) = Xk_Eskf;
        Pestimado_Eskf(:,t) = [Pk_Eskf(1,1); Pk_Eskf(2,2); Pk_Eskf(3,3)];
        time_Eskf(t) = timek;
    
        
    tic   
    [Xk_Ukf, Pk_Ukf] = UkfTask(balizas, Q, R_angle, R_distance, uk, Euk_local, q_Ukf, sv_aug_Ukf, sv_aug_sigma_Ukf, lambda_Ukf, wm_Ukf, wc_Ukf, Xk_Ukf, Pk_Ukf, Zk, aux);
    timek = toc;
    
        % REGISTRO:
        Xestimado_Ukf(:,t) = Xk_Ukf(1:3);
        Pestimado_Ukf(:,t) = [Pk_Ukf(1,1); Pk_Ukf(2,2); Pk_Ukf(3,3)]; 
        time_Ukf(t) = timek;
     
        
    tic    
    [XkAug_Uskf, PkAug_Uskf] = UskfTask(balizas, R_angle, R_distance, uk, b_Uskf, nb_Uskf, sv_aug_Uskf, sv_aug_sigma_Uskf, lambda_Uskf, wm_Uskf, wc_Uskf, XkAug_Uskf, PkAug_Uskf, Zk, aux);
    timek = toc;
    
        % REGISTRO:
        Xestimado_Uskf(:,t) = XkAug_Uskf(1:3);
        Pestimado_Uskf(:,t) = [PkAug_Uskf(1,1); PkAug_Uskf(2,2); PkAug_Uskf(3,3)]; 
        time_Uskf(t) = timek;
    
    
        
    % COMPROBACIÓN OBJETIVO:
    target_reached = pointReached(Xkreal(1:2)', path(size(path,1),:), tolerance);
    point_reached = pointReached(Xkreal(1:2)', path(1,:), tolerance);
    if point_reached == 1
        path(1,:) = [];
    end
    
    
    % COMPROBACIÓN LÍMITE DEL MAPA:
    oldWorld = world;
    [change, world, transf] = getWorld(Xkreal, puertas, world);
    if change > 0 || target_reached
        
        t_f = t;
        
        buffer(fig_id).wide = wide;
        buffer(fig_id).long = long;
        buffer(fig_id).obstacles = obstacles;
        buffer(fig_id).balizas = balizas;
        buffer(fig_id).puertas = puertas;
        buffer(fig_id).oldWorld = oldWorld;
        buffer(fig_id).t_i = t_i;
        buffer(fig_id).t_f = t_f;
        
        fig_id = fig_id + 1;
        
        if change > 0
            t_i = t + 1;
            
            Xk_Ekf = Xk_Ekf + transf;
            Xk_Eskf = Xk_Eskf + transf;
            Xk_Ukf = Xk_Ukf + transf;
            XkAug_Uskf = XkAug_Uskf + [transf; zeros(nb_Uskf,1)];

            Xkreal = Xkreal + transf;
            apoloPlaceMRobot(robot, [Xkreal(1) Xkreal(2) 0], Xkreal(3), world);
            apoloUpdate(world);
            [wide, long, obstacles, balizas, puertas] = initWorld(world);

            path = path + transf(1:2)';
        end
    end
    
end  


% CÁLCULO DEL ERROR
error_Ekf = abs(Xreal - Xestimado_Ekf);
error_Ekf(3,:) = fixAngleRad(error_Ekf(3,:));
error_mean_Ekf = mean(error_Ekf,2);

error_Eskf = abs(Xreal - Xestimado_Eskf);
error_Eskf(3,:) = fixAngleRad(error_Eskf(3,:));
error_mean_Eskf = mean(error_Eskf,2);

error_Ukf = abs(Xreal - Xestimado_Ukf);
error_Ukf(3,:) = fixAngleRad(error_Ukf(3,:));
error_mean_Ukf = mean(error_Ukf,2);

error_Uskf = abs(Xreal - Xestimado_Uskf);
error_Uskf(3,:) = fixAngleRad(error_Uskf(3,:));
error_mean_Uskf = mean(error_Uskf,2);


% TIEMPO TOTAL:
totalTime_Ekf = sum(time_Ekf);
totalTime_Eskf = sum(time_Eskf);
totalTime_Ukf = sum(time_Ukf);
totalTime_Uskf = sum(time_Uskf);

% TIEMPO MEDIO:
meanTime_Ekf = mean(time_Ekf);
meanTime_Eskf = mean(time_Eskf);
meanTime_Ukf = mean(time_Ukf);
meanTime_Uskf = mean(time_Uskf);

% REPRESENTACIÓN DE ESTADO X
if exist('folder', 'var')
    switch folder
        case 'EKFvsESKF'
            compare = [1 1 0 0];
        case 'UKFvsUSKF'
            compare = [0 0 1 1];
        case 'ESKFvsUSKF'
            compare = [0 1 0 1];
        case 'AllvsAll'
            compare = [1 1 1 1];
        otherwise
            error('No such folder')
    end
else 
    compare = [1 1 1 1];
end

for i = 1:size(buffer,2) 
    plotXreal(buffer(i).wide, buffer(i).long, buffer(i).obstacles, buffer(i).balizas, buffer(i).puertas, buffer(i).oldWorld, Xreal(:,buffer(i).t_i:buffer(i).t_f))
    if compare(1)
        plotX(Xestimado_Ekf(:,buffer(i).t_i:buffer(i).t_f), i, 'EKF')
    end
    if compare(2)
        plotX(Xestimado_Eskf(:,buffer(i).t_i:buffer(i).t_f), i, 'ESKF')
    end
    if compare(3)
        plotX(Xestimado_Ukf(:,buffer(i).t_i:buffer(i).t_f), i, 'UKF')
    end
    if compare(4)
        plotX(Xestimado_Uskf(:,buffer(i).t_i:buffer(i).t_f), i, 'USKF')
    end
end

% REPRESENTACIÓN DE INCERTIDUMBRE P
if compare(1)
    plotP(Pestimado_Ekf(:,t_i:t_f), fig_id, 'EKF')
end
if compare(2)
    plotP(Pestimado_Eskf(:,t_i:t_f), fig_id, 'ESKF')
end
if compare(3)
    plotP(Pestimado_Ukf(:,t_i:t_f), fig_id, 'UKF')
end
if compare(4)
    plotP(Pestimado_Uskf(:,t_i:t_f), fig_id, 'USKF')
end

% REPRESENTACION DE ERROR Y TIEMPO
% plotTyE(time_Ekf, speed, error_Ekf, fig_id, 'EKF')
% plotTyE(time_Eskf, speed, error_Eskf, fig_id, 'ESKF')
% plotTyE(time_Ukf, speed, error_Ukf, fig_id, 'UKF')
% plotTyE(time_Uskf, speed, error_Uskf, fig_id, 'USKF')