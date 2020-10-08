function [rad] = fixAngleRad(rad)
    for i = 1:length(rad)
        if(rad(i) > pi)
            while(rad(i) > pi)
                rad(i) = rad(i) - 2*pi;
            end
        elseif (rad(i) < -pi)
            while(rad(i) < -pi)
                rad(i) = rad(i) + 2*pi;
            end
        end
    end
end

