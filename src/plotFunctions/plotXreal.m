function [] = plotXreal(wide, long, obstacles, balizas, puertas, world, Xreal)


% FIGURA 1: Evolución del estado X
f = figure();
f.WindowState = 'maximized';

    % Subfigura 1: Trayectoria en el mapa
    axis([-0.1 wide+0.1 -0.1 long+0.1])
    hold on

        % Balizas
        for i = 1:length(balizas)
            plot(balizas(i,1),balizas(i,2),'sk');
        end

        % Obstáculos
        for i = 1:size(obstacles,1)
            plot([obstacles(i,1), obstacles(i,3)],[obstacles(i,2), obstacles(i,2)], 'r')
            hold on
            plot([obstacles(i,1), obstacles(i,3)],[obstacles(i,4), obstacles(i,4)], 'r')
            hold on
            plot([obstacles(i,1), obstacles(i,1)],[obstacles(i,2), obstacles(i,4)], 'r')
            hold on
            plot([obstacles(i,3), obstacles(i,3)],[obstacles(i,2), obstacles(i,4)], 'r')
            hold on
        end  

        % Puertas
        for i = 1:size(puertas,1)
            plot([puertas(i,1), puertas(i,3)],[puertas(i,2), puertas(i,2)], 'm')
            hold on
            plot([puertas(i,1), puertas(i,3)],[puertas(i,4), puertas(i,4)], 'm')
            hold on
            plot([puertas(i,1), puertas(i,1)],[puertas(i,2), puertas(i,4)], 'm')
            hold on
            plot([puertas(i,3), puertas(i,3)],[puertas(i,2), puertas(i,4)], 'm')
            hold on
        end  
        
    p1 = plot(Xreal(1,:),Xreal(2,:),'.k');

    xlabel ('X (m)')
    ylabel ('Y (m)')
    title ({['Mapa: ', world]; 'Trayectoria seguida'})
    legend(p1,'Real')
    legend('boxoff')
    hold on

   
end

