function [XkAug, PkAug] = UskfTask(balizas, R_angle, R_distance, uk, b, nb, sv_aug, sv_aug_sigma, lambda, wm, wc, XkAug, PkAug, Zk, aux)
  
% 1. OBSERVACIÓN:

    % Ya hecho en control.m. Devuelve Zk
    

% 2. PREDICCIÓN:
    % A. Estado:   (no depende de los sensores)
    
    % Matriz de sigma points Xsig+(k|k):
    L = chol(PkAug, 'lower');
    c = sqrt(sv_aug + lambda);
    
    XsigPredAug = zeros(sv_aug,sv_aug_sigma);
    XsigPredAug(:,1) = XkAug;
    for i = 2:(sv_aug+1)
        XsigPredAug(:,i) = XkAug + c * L(:,i-1);
        XsigPredAug(3,i) = fixAngleRad(XsigPredAug(3,i));
        
        XsigPredAug(:,i+sv_aug) = XkAug - c * L(:,i-1);
        XsigPredAug(3,i+sv_aug) = fixAngleRad(XsigPredAug(3,i+sv_aug));
    end
    
    % Predecimos los sigma points Xsig+(k+1|k):
        % Calculamos las predicciones sin corregir (biased) y corregidas (unbiased):
        XsigPred_biased = zeros(sv_aug,sv_aug_sigma);
        for i = 1:sv_aug_sigma
            XsigPred_biased(:,i) = [(XsigPredAug(1,i) + uk(1)*cos(XsigPredAug(3,i)) - uk(2)*sin(XsigPredAug(3,i)));
                                    (XsigPredAug(2,i) + uk(1)*sin(XsigPredAug(3,i)) + uk(2)*cos(XsigPredAug(3,i)));
                                    (XsigPredAug(3,i) + uk(3));
                                    (XsigPredAug(4:nb+3,i))]; 

            XsigPred_biased(3,i) = fixAngleRad(XsigPred_biased(3,i));
        end
        
%         uk_unbiased = uk + Euk_local;
        uk_unbiased = uk + b(1:3);
        uk_unbiased(3,1) = fixAngleRad(uk_unbiased(3,1));
        
        XsigPred_unbiased = zeros(sv_aug,sv_aug_sigma);
        for i = 1:sv_aug_sigma
%             uk_unbiased = uk + XsigPredAug(4:6,i);
%             uk_unbiased(3,1) = fixAngleRad(uk_unbiased(3,1));
            
            XsigPred_unbiased(:,i) = [(XsigPredAug(1,i) + uk_unbiased(1)*cos(XsigPredAug(3,i)) - uk_unbiased(2)*sin(XsigPredAug(3,i)));
                                      (XsigPredAug(2,i) + uk_unbiased(1)*sin(XsigPredAug(3,i)) + uk_unbiased(2)*cos(XsigPredAug(3,i)));
                                      (XsigPredAug(3,i) + uk_unbiased(3));
                                      (XsigPredAug(4:nb+3,i))]; 

            XsigPred_unbiased(3,i) = fixAngleRad(XsigPred_unbiased(3,i));
        end
        
    XsigPred = XsigPred_biased;
        
    % Predecimos el estado X(k+1|k) a partir de los sigma points:
    XkAug_mean = zeros(sv_aug,1);
    for i = 1:sv_aug_sigma
        XkAug_mean = XkAug_mean + wm(i)*XsigPred(:,i);
    end
    XkAug_mean(3,1) = fixAngleRad(XkAug_mean(3,1));
    XkAugp = XkAug_mean;
    
    % Predecimos la incertidumbre del estado P(k+1|k):
    Pyy = zeros(sv_aug,sv_aug);
    for i = 1:sv_aug_sigma
        diff = XsigPred(:,i) - XkAugp;
        diff(3,1) = fixAngleRad(diff(3,1));
        Pyy = Pyy + wc(i)*(diff * diff');
    end
    PkAugp = Pyy;
    
    if PkAugp(1,1) < 0 || PkAugp(2,2) < 0 || PkAugp(3,3) < 0
        error('Pk negativo')
    end

    
    % B. Medida:   (depende de los sensores)
    
    % Predecimos el valor de z para cada sigma point Zsig(k+1):
    results = zeros(2*length(aux.id), sv_aug_sigma);
    diag_R = [];
    
    for j = 1:length(aux.id)
        for i = 1:sv_aug_sigma
            
            results((2*j-1):(2*j),i) = [sqrt((balizas(aux.id(j),1)-XsigPred(1,i))^2 + (balizas(aux.id(j),2)-XsigPred(2,i))^2)+XsigPred(3+nb-1,i);
                                        atan2((balizas(aux.id(j),2)-XsigPred(2,i)),(balizas(aux.id(j),1)-XsigPred(1,i)))-XsigPred(3,i)+XsigPred(3+nb,i)];

            results(2*j,i) = fixAngleRad(results(2*j,i));
        end
        diag_R = [diag_R R_distance R_angle];
    end
    Zsig = results;
    Rk = diag(diag_R);

    % Calculamos el valor ponderado de Z(k+1)
    if size(Zk) > 0
        Zk_mean = zeros(2*length(aux.id),1);
        for i = 1:sv_aug_sigma
            Zk_mean = Zk_mean + wm(i)*Zsig(:,i);
            Zk_mean(2,1) = fixAngleRad(Zk_mean(2,1));
        end
        Zkp = Zk_mean;
    end
    
    
% 3. COMPARACIÓN:

    if size(Zk) > 0
        
        Vk = Zk - Zkp;
        for i=1:length(Vk)
            if (mod(i,2) == 0) && (Vk(i) > pi)
                Vk(i,1) = fixAngleRad(Vk(i,1)); 
            end
        end
            
%             % Validación de la medida:
%             for i=1:(length(Vk)/2)
%                 mdDistance = abs(Vk(2*i-1))/sqrt(R_distance);
%                 mdAngle = abs(Vk(2*i))/sqrt(R_angle);
%                 
%                 if (mdDistance > TMdistance || mdAngle > TMangle)
%                     Vk(2*i-1) = 0;
%                     Vk(2*i) = 0;
%                 end
%             end
       
        Sk = zeros(2*length(aux.id),2*length(aux.id));
        for i = 1:sv_aug_sigma
            diff_z = Zsig(:,i) - Zkp;
            Sk = Sk + wc(i)*(diff_z * diff_z');
        end
        Sk = Rk + Sk;
        
        Pxz = zeros(sv_aug,2*length(aux.id));
        for i = 1:sv_aug_sigma
            diff_x = XsigPred(:,i) - XkAugp;
            diff_x(3,1) = fixAngleRad(diff_x(3,1));

            diff_z = Zsig(:,i) - Zkp;
            Pxz = Pxz + wc(i)*(diff_x * diff_z');
        end
        
        Wk = Pxz / Sk;
    
    else
        Vk = 0;
        Sk = 0;
        Wk = zeros(sv_aug,1);
    
    end
    
    
% 4. CORRECCIÓN:

    Wxk = Wk(1:3,:);
    Wbk = Wk(4:sv_aug,:);
            
    XkAug = XkAugp + [Wxk * Vk; 
                      zeros(nb,1)];
    XkAug(3,1) = fixAngleRad(XkAug(3,1));
    
    PkAug = PkAugp - [Wxk*Sk*Wxk'     Wxk*Sk*Wbk';
                      Wbk*Sk*Wxk'     zeros(nb,nb)];

end

