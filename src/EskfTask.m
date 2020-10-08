function [Xk, Pk, Pxbk] = EskfTask(balizas, Q, R_angle, R_distance, b, Bk, uk, Euk_local, nb, Xk, Pk, Pxbk, Zk, aux)

Xk_1 = Xk;
Pk_1 = Pk;

% 1. OBSERVACI�N:

    % Ya hecho en control.m. Devuelve Zk
    

% 2. PREDICCI�N:
    % A. Estado:   (no depende de los sensores)
                                  
        % Calculamos la odometria sin corregir (biased) y corregida (unbiased):
        Xkp_biased = [(Xk_1(1) + uk(1)*cos(Xk_1(3)) - uk(2)*sin(Xk_1(3)));
                      (Xk_1(2) + uk(1)*sin(Xk_1(3)) + uk(2)*cos(Xk_1(3)));
                      (Xk_1(3) + uk(3))];
        Xkp_biased(3,1) = fixAngleRad(Xkp_biased(3,1));
        
        uk_unbiased = uk + Euk_local;
        uk_unbiased(3,1) = fixAngleRad(uk_unbiased(3,1));

        Xkp_unbiased = [(Xk_1(1) + uk_unbiased(1)*cos(Xk_1(3)) - uk_unbiased(2)*sin(Xk_1(3)));
                        (Xk_1(2) + uk_unbiased(1)*sin(Xk_1(3)) + uk_unbiased(2)*cos(Xk_1(3)));
                        (Xk_1(3) + uk_unbiased(3))]; 
        Xkp_unbiased(3,1) = fixAngleRad(Xkp_unbiased(3,1));
       
    Xkp = Xkp_biased;
    
%     Xkp = Xkp_unbiased;     uk = uk_unbiased;
            
    F = [1 0 -uk(1)*sin(Xk_1(3))-uk(2)*cos(Xk_1(3));
         0 1 uk(1)*cos(Xk_1(3))-uk(2)*sin(Xk_1(3));
         0 0 1];
        
    G = [cos(Xk_1(3)) -sin(Xk_1(3)) 0;
         sin(Xk_1(3)) cos(Xk_1(3))  0;
         0            0             1];

    Pkp = F * Pk_1 * F' + G * Q * G';
    Bkp = Bk;
    Pxbkp = F * Pxbk;
    
        if Pkp(1,1) < 0 || Pkp(2,2) < 0 || Pkp(3,3) < 0
            error('Pk negativa')
        end
    
    
    % B. Medida:   (depende de los sensores)

    Zkp = [];
    H = [];
    Hb = [];
    diag_R = [];
    
    for i = 1:length(aux.id)
        
        Pos = Xkp_biased;
        distance = sqrt((balizas(aux.id(i),1)-Pos(1))^2+(balizas(aux.id(i),2)-Pos(2))^2);
        
        Zkp = [Zkp; 
               distance;
               fixAngleRad(atan2((balizas(aux.id(i),2)-Pos(2)),(balizas(aux.id(i),1)-Pos(1)))-Pos(3))];
      
        H = [H; 
             -(balizas(aux.id(i),1)-Pos(1))/distance   -(balizas(aux.id(i),2)-Pos(2))/distance    0;
              (balizas(aux.id(i),2)-Pos(2))/distance^2 -(balizas(aux.id(i),1)-Pos(1))/distance^2 -1];     
        
          
        Pos = Xkp_unbiased;
        distance = sqrt((balizas(aux.id(i),1)-Pos(1))^2+(balizas(aux.id(i),2)-Pos(2))^2);
        
%         Hb = [Hb;
%               1 0;
%               0 1];
        
%         Hb = [Hb;
%               0 0 0 1 0;
%               0 0 0 0 1];

        Hb = [Hb;
              ((Pos(1)-balizas(aux.id(i),1))*cos(Xk_1(3))+(b(1)*sin(Xk_1(3))-balizas(aux.id(i),2)+Pos(2))*sin(Xk_1(3))+b(1)*cos(Xk_1(3))^2)/distance      ((Pos(2)-balizas(aux.id(i),2))*cos(Xk_1(3))+(b(2)*sin(Xk_1(3))+balizas(aux.id(i),1)-Pos(1))*sin(Xk_1(3))+b(2)*cos(Xk_1(3))^2)/distance     0 1 0;
              (-(Pos(2)-balizas(aux.id(i),2))*cos(Xk_1(3))+(b(2)*sin(Xk_1(3))+balizas(aux.id(i),1)-Pos(1))*sin(Xk_1(3))-b(2)*cos(Xk_1(3))^2)/distance^2   ((Pos(1)-balizas(aux.id(i),1))*cos(Xk_1(3))+(b(1)*sin(Xk_1(3))-balizas(aux.id(i),2)+Pos(2))*sin(Xk_1(3))+b(1)*cos(Xk_1(3))^2)/distance^2  -1 0 1];        
          
        diag_R = [diag_R R_distance R_angle];
        
    end
    
    Rk = diag(diag_R);
    
    
% 3. COMPARACI�N:

    if size(Zk) > 0
        
        Vk = Zk - Zkp;
        for i=1:length(Vk)
            if (mod(i,2) == 0) && (Vk(i) > pi)
                Vk(i,1) = fixAngleRad(Vk(i,1)); 
            end
        end
            
%             % Validaci�n de la medida:
%             for i=1:(length(Vk)/2)
%                 mdDistance = abs(Vk(2*i-1))/sqrt(R_distance);
%                 mdAngle = abs(Vk(2*i))/sqrt(R_angle);
%                 
%                 if (mdDistance > TMdistance || mdAngle > TMangle)
%                     Vk(2*i-1) = 0;
%                     Vk(2*i) = 0;
%                 end
%             end
        
        Sk = (H * Pkp * H') + (Hb * Pxbkp' * H') + (H * Pxbkp * Hb') + (Hb * Bkp * Hb') + Rk;
        
        Wk = (Pkp * H' + Pxbkp * Hb') / Sk;
    
    else
        H = zeros(1,3);
        Hb = zeros(1,nb);
        Vk = 0;
        Sk = 0;
        Wk = zeros(3,1);
        
    end
    
    
% 4. CORRECCI�N:
        
    Xk = Xkp + Wk * Vk;
    Xk(3,1) = fixAngleRad(Xk(3,1));
        
    Pk = Pkp - Wk * H * Pkp - Wk * Hb * Pxbkp';
    Bk = Bkp;
    Pxbk = Pxbkp - Wk * H * Pxbkp - Wk * Hb * Bkp;

end
