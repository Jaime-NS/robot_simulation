function [x] = normalNoise(media, desv)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
persistent vale N2
if isempty(vale)
    vale = 0;
end
if vale
    y = N2;
    vale = 0;
else
    u1 = uniformesin0;
    u2 = uniforme;

    N1 = sqrt(-2 * log(u1)) * cos (2 * pi* u2);
    N2 = sqrt(-2 * log(u1)) * sin (2 * pi* u2);
    y = N1;
    vale = 1;
end
x = y * desv + media;
end

function u = uniformesin0
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

persistent x0

if isempty(x0)
    x0 = 10000*sum(clock);        % semilla (numero aleatorio)
end
x0 = rem(314159269 * x0 + 453806245, 2^31);
x0 = x0 + 1;
u = x0/2^31;                         % devuelve un numero en el intervalo (0,1]
end

function u = uniforme
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

persistent x0

if isempty(x0)
    x0 = 36514680*sum(clock);        % semilla (numero aleatorio)
end
x0 = rem(314159269 * x0 + 453806245, 2^31);
u = x0/2^31;                         % devuelve un numero en el intervalo [0,1)
end


