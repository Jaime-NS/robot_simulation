function [] = plotX(Xestimado, fig_id, filter)

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
        
% FIGURA 1: Evolución del estado X
figure(fig_id)

    % Subfigura 1: Trayectoria en el mapa
    hold on    
    plot(Xestimado(1,:),Xestimado(2,:), strcat('.', color), 'DisplayName', filter);
    hold on
     
end

