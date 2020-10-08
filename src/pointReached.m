function [ret] = pointReached(q1,q2,tolerance)

    distance = sqrt((q1(1)-q2(1))^2 + (q1(2)-q2(2))^2);
    
    if distance <= tolerance
        ret = 1;
    else
        ret = 0;
    end
end

