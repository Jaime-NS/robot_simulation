function [] = plotTyE(time, speed, error, fig_id, filter)

switch filter
    case 'EKF'
        color = 'r';
    case 'ESKF'
        color = 'b';
    case 'UKF'
        color = 'y';
    case 'USKF'
        color = 'g';
    otherwise
        error('No such filter')
end

% FIGURA
figure(2*fig_id-1)

    % Subfigura 1: Relación del error y la velocidad
    subplot(2,2,1)
    plot(speed, error(1,:), strcat('.', color), 'DisplayName', filter)
    xlabel ('Error (m)')
    ylabel ('Velocidad (m/s)')
    title ('Relación velocidad-error en x')
    legend('boxoff')
    hold on
    
    % Subfigura 1: Relación del error y la velocidad
    subplot(2,2,2)
    plot(speed, error(2,:), strcat('.', color), 'DisplayName', filter)
    xlabel ('Error (m)')
    ylabel ('Velocidad (m/s)')
    title ('Relación velocidad-error en y')
    legend('boxoff')
    hold on
    
    % Subfigura 1: Relación del error y la velocidad
    subplot(2,2,3)
    plot(speed, error(3,:), strcat('.', color), 'DisplayName', filter)
    xlabel ('Error (rad)')
    ylabel ('Velocidad (m/s)')
    title ('Relación velocidad-error en \theta')
    legend('boxoff')
    hold on

    % Subfigura 4: Tiempo consumido
    subplot(2,2,4);
    plot(time(:),color, 'DisplayName', filter);
    xlabel ('t (muestras)')
    ylabel ('t ejecución(t/muestra)')
    title ('Tiempo de ejecución por iteración')
    legend('boxoff')
    hold on
    
end