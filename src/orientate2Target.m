function [secuence] = orientate2Target(q1, q2, orientation)
    
    speed_ang = pi;
    
    v1 = [cos(orientation) sin(orientation)];
    v2 = [q2(1)-q1(1) q2(2)-q1(2)];
    
    alfa1 = atan2(v1(2), v1(1));
    alfa2 = atan2(v2(2), v2(1));
    
    angle = alfa2 - alfa1;
    
    secuence = [0 speed_ang angle];
    
end

