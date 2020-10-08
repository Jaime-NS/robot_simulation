function [change, world, transf] = getWorld(Xk, puertas, world)
    
    change = 0;
    
    switch (world)
        case 'World 1'
        case 'mobile robots'
            world_i = 0;
        case 'computer vision'
            world_i = 1;
        case 'main hallway'
            world_i = 2;
        case 'main receiver'
            world_i = 3;
        case 'mr-fm hallway'
            world_i = 4;
        case 'fm office'
            world_i = 6;
        otherwise
            error('Map not existent')
    end
    
    for i = 1:size(puertas,1)
        if (Xk(1) >= puertas(i,1) && Xk(1) <= puertas(i,3) && Xk(2) >= puertas(i,2) && Xk(2) <= puertas(i,4))
            world_f = puertas(i,5);
            if isnan(world_f)
                error('Map not existent')
            else
                change = world_i*10 + world_f;
                break
            end
        else
            change = 0;
        end
    end
    
    switch (change)
        case 0
            transf = [0 0 0]';
        
        case 10
            world = 'mobile robots';
            transf = [0.18 -11.48 0]';
        case 40
            world = 'mobile robots';
            transf = [-1.60 0 0]';
               
        case 01
            world = 'computer vision';
            transf = [-0.18 11.48 0]';
        case 21
            world = 'computer vision';
            transf = [0 -10.59 0]';
            
        case 12
            world = 'main hallway';
            transf = [0 10.59 0]';
        case 32
            world = 'main hallway';
            transf = [0 -6.10 0]';     
            
        case 23
            world = 'main receiver';
            transf = [0 6.10 0]';
            
        case 04
            world = 'mr-fm hallway';
            transf = [1.60 0 0]';
        case 64
            world = 'mr-fm hallway';
            transf = [-3.10 0 0]';
            
        case 46
            world = 'fm office';
            transf = [3.10 0 0]';
            
        otherwise
            error('Impossible')
    end
       
end

