function [wide, long, obstacles, balizas, puertas] = initWorld(world)

wide = 10.3;
    
    if strcmp(world, 'mobile robots')
        % MAPA
%         wide = 8.95;
        long = 4.58;

        % Obstaculos definidos como [xmin ymin xmax ymax]
        obstacles = [];
        
        
        % Posición de las balizas
        balizas = [0.4 0.27;
                   1.59 0.27;
                   0.8 1.87;
                   0.8 3.06;
                   1.99 1.07;
                   3.58 1.47;
                   NaN NaN;
                   5.17 1.47;
                   5.17 3.06;
                   7.17 1.87;
                   7.17 3.06];
               
        puertas = [1 -0.1 2.2 0 1;
                   -0.1 0.65 0 1.45 4];

    elseif strcmp(world, 'computer vision')
        % MAPA
%         wide = 10.3;
        long = 11.48;

        % Obstaculos definidos como [xmin ymin xmax ymax]
        obstacles = [2.1 7.78 2.4 9.57;
                     2.1 9.57 2.4 11.36;
                     3.73 7.78 4.03 9.57;
                     3.73 9.57 4.03 11.36;
                     5.36 7.78 5.66 9.57;
                     5.36 9.57 5.66 11.36;
                     6.16 7.78 6.46 9.57;
                     6.16 9.57 6.46 11.36;
                     0 3 0.6 9.7];
        
        
        % Posición de las balizas
        balizas = [0.96 11.46;
                   4.64 11.46;
                   0.92 0.05;
                   0.05 2.09;
                   0.05 4.55;
                   0.05 7.13;
                   0.05 9.85];
                          
        puertas = [0.82 11.48 2.02 11.58 0;
                   -0.1 0.65 0 2.02 NaN;
                   0.3 -0.1 1.5 0 2];
               
    elseif strcmp(world, 'mr-fm hallway')
        % MAPA
%         wide = 1.6;
        long = 4;

        % Obstaculos definidos como [xmin ymin xmax ymax]
        obstacles = [];
        
        
        % Posición de las balizas
        balizas = [];
                          
        puertas = [-0.1 0.6 0 1.4 6;
                   0.6 -0.1 1.4 0 NaN;
                   1.6 0.65 1.7 1.45 0;
                   0.6 4 1.4 4.1 NaN];
       
    elseif strcmp(world, 'fm office')
        % MAPA
%         wide = 3.1;
        long = 4.2;

        % Obstaculos definidos como [xmin ymin xmax ymax]
        obstacles = [0.98 1.47 1.8 3.27;
                     0 3.27 1.8 4.2];
        
        
        % Posición de las balizas
        balizas = [];
                          
        puertas = [3.1 0.6 3.2 1.4 4];
        
    elseif strcmp(world, 'main hallway')
        % MAPA
%         wide = 1.8;
        long = 10.59;

        % Obstaculos definidos como [xmin ymin xmax ymax]
        obstacles = [0 4.48 0.42 7.6;
                     0 9.2 0.4 10.13];
        
        
        % Posición de las balizas
        balizas = [1.7 2.75;
                   0.1 3.76;
                   1.7 5.75;
                   0.1 7.68;
                   1.7 9.75];
                          
        puertas = [-0.1 3 0 3.8 NaN;
                   -0.1 8.28 0 9.08 NaN;
                   0.3 -0.1 1.5 0 3;
                   1.8 1.35 1.9 2.15 NaN;
                   1.8 9.05 1.9 9.85 NaN;
                   0.3 10.59 1.5 10.69 1];
               
    elseif strcmp(world, 'main receiver')
        % MAPA
%         wide = 2.56;
        long = 6.1;

        % Obstaculos definidos como [xmin ymin xmax ymax]
        obstacles = [0 0 1.17 3.9];
        
        
        % Posición de las balizas
        balizas = [];
                          
        puertas = [-0.1 4.46 0 4.36 NaN;
                   0.3 6.1 1.5 6.2 2;
                   2.56 4.8 2.66 5.68 NaN];
               
    elseif strcmp(world, 'World 1')
        % MAPA
        wide = 62;
        long = 58;
        
        % Obstaculos definidos como [xmin ymin xmax ymax]
        obstacles = [0 3 10 58;
                     10 49 21 49.5;
                     25 49 47 49.5;
                     51 49 62 49.5;
                     35.5 49.5 36.5 58];
                 
        % Posición de las balizas
        balizas = [0.1 0.1;
                   0.1 2.9;
                   10.1 0.1;
                   10.1 2.9;
                   10.1 17;
                   10.1 32;
                   10.1 47;
                   10.1 54;
                   16.1 9.5;
                   16.1 24.5;
                   16.1 39.5;
                   22.1 9.5;
                   22.1 24.5;
                   22.1 39.5;
                   28.1 9.5;
                   28.1 24.5;
                   28.1 39.5;
                   34.1 9.5;
                   34.1 24.5;
                   34.1 39.5;
                   40.1 9.5;
                   40.1 24.5;
                   40.1 39.5;
                   46.1 9.5;
                   46.1 24.5;
                   46.1 39.5;
                   52.1 9.5;
                   52.1 24.5;
                   52.1 39.5;
                   58.1 9.5;
                   58.1 24.5;
                   58.1 39.5;
                   27 0.1;
                   39 0.1;
                   51 0.1;
                   61.9 0.1;
                   27 15.1;
                   39 15.1;
                   51 15.1;
                   61.9 15.1;
                   27 30.1;
                   39 30.1;
                   51 30.1;
                   61.9 30.1;
                   27 45.1;
                   39 45.1;
                   51 45.1;
                   61.9 45.1;
                   21.1 49.25;
                   47.1 49.25;
                   23 57.9;
                   49 57.9;
                   35.4 54;
                   36.6 54;
                   61.9 54];

        puertas = [];
    
    end
end

