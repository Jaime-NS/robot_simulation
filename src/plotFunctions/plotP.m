function [] = plotP(Pestimado, fig_id, filter)

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
  
    
% FIGURA 2: Evolución de la incertidumbre P
f = figure(fig_id);
f.WindowState = 'maximized';
   
        % Subfigura 1: Evolución en x
    subplot(1,3,1);
    plot(Pestimado(1,:),color, 'DisplayName', filter, 'LineWidth',1.5);
    xlabel ('t (muestras)')
    ylabel ('X (m)')
    xlim([0 inf])
%     ylim([0 1e-1])
    title ('Incertidumbre en x')
    legend('boxoff')
    hold on

    % Subfigura 2: Evolución en y
    subplot(1,3,2);
    plot(Pestimado(2,:),color, 'DisplayName', filter, 'LineWidth',1.5);
    xlabel ('t (muestras)')
    ylabel ('Y (m)')
    xlim([0 inf])
%     ylim([0 1e-1])
    title ('Incertidumbre en y')
    legend('boxoff')
    hold on

    % Subfigura 3: Evolución en theta
    subplot(1,3,3);
    plot(Pestimado(3,:),color, 'DisplayName', filter, 'LineWidth',1.5);
    xlabel ('t (muestras)')
    ylabel ('\theta (rad)')
    xlim([0 inf])
%     ylim([0 1e-1])
    title ('Incertidumbre en \theta')
    legend('boxoff')
    hold on
       
end

