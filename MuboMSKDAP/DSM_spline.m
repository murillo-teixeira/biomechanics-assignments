function [Spline_0th] = DSM_spline(t,z,order)
%
% This function: Evaluates a Cubic spline from an input dataset. 
%                The Spline is stored as a structure with the same
%                arrangement of the Spline that can be obtained by using 
%                the standard Matlab function 'spline' 
%
% The input to the function is :
%    t     - Vector with the parameter values
%    z     - Vector with the function values
%    type  - Type of Cubic Spline (1 - Natural Spline; 2 - Periodic Spline)
%    order - Order of the spline (=4)Cubic; (=6)Quintic)
%
% The output of the function is :
%    spline_0th - Structure with the cubic spline
%
%%
%Global memory assignments
global spline_type
%
%% ... General dimensions
pieces  = length(t)-1;
%
%... Create the Spline structure
Spline_0th.form   = 'pp';
Spline_0th.breaks = t;
Spline_0th.coefs  = zeros(pieces,order);
Spline_0th.pieces = pieces;
Spline_0th.order  = order;
Spline_0th.dim    = 1;
%
%... Build the systems of equations to be solved
n = order-2;
A = zeros(n*Spline_0th.pieces,n*Spline_0th.pieces);
b = zeros(n*Spline_0th.pieces,1);
%
%% ... Continuity conditions for the Spline pieces
for i=1:pieces-1
    h  = t(i+1)-t(i);
    h2 = h*h;
%
    switch order
        case 4
            l1 = 2*i-1;
            l2 = l1+1;
            c1 = l1;
            c2 = l1+3;
            A(l1:l2,c1:c2) = [h 2*h2/3  0 h2/3;...
                              1   h    -1  h];
            b(l1:l2,1)     = [z(i+1)-z(i); 0.0];
        case 6
            h3 = h2*h;
            h4 = h3*h;
            l1 = 4*i-3;
            l2 = l1+3;
            c1 = l1;
            c2 = l1+7;
            A(l1:l2,c1:c2) = [h   h2   h3  4*h4/5  0  0  0  h4/5;...
                              1  2*h  3*h2  3*h3  -1  0  0   h3; ...
                              0   2   6*h   8*h2   0 -2  0  4*h2; ...
                              0   0    6    12*h   0  0 -6  12*h];
            b(l1:l2,1)     = [z(i+1)-z(i); 0; 0; 0];
    end
end
%
%% ... Spline end conditions
l1 = l2+1;
dz = z(pieces+1)-z(pieces);
h  = t(pieces+1)-t(pieces);
h2 = h*h;
c2 = (order-2)*pieces;
%
%... Natural Cubic Spline end conditions
if (spline_type == 1)
%
    switch order
        case 4
            A(l1,2)        = 1;
            b(l1,1)        = 0;
%
            l1 = l1+1;
            c1 = c2-1;
            A(l1,c1:c2)    = [h 2*h2/3];
            b(l1,1)        = dz;
        case 6
            h3 = h2*h;
            h4 = h3*h;
            l2 = l1+1;
            A(l1:l2,3:4)   = eye(2);
            b(l1:l2,1)     = 0;
%
            l1 = l2+1;
            l2 = l1+1;
            c1 = c2-3;
            A(l1:l2,c1:c2) = [0  0   6   12*h;...
                              h  h2  h3  4*h4/5];
            b(l1:l2,1)     = [0; dz];
           
    end
%
%... Periodic Cubic Spline end conditions
elseif (spline_type == 2)
    switch order
        case 4
            l2 = l1+1;
            c1 = c2-1;
            A(l1:l2,c1:c2) = [h   2*h2/3;...
                              1   h];
            b(l1:l2,1)     = [dz; 0];
%
            c1 = 1;
            c2 = 2;
            A(l1:l2,c1:c2) = [ 0  h2/3;...
                              -1  h];
        case 6
            l2 = l1+3;
            c1 = c2-3;
            A(l1:l2,c1:c2) = [h   h2   h3    4*h4/5;...
                              1   2*h  3*h2  3*h3; ...
                              0   2    6*h   8*h2; ...
                              0   0    6     12*h];
            b(l1:l2,1)     = [dz; 0 ;  0;    0];
%
            c1 = 1;
            c2 = 4;
            A(l1:l2,c1:c2) = [ 0  0  0  h4/5; ...
                              -1  0  0  h3; ...
                               0 -2  0  4*h2; ...
                               0  0 -6  12*h];
    end
end
%
%% ... Solve system of equations to obtain Spline Coefficients
x = A\b;
%
%% ... Store the Spline coefficients in proper vector
for i=1:pieces-1
    h  = t(i+1)-t(i);
    l1 = n*i;
    switch order
        case 4
            d  = [(x(l1+2,1)-x(l1,1))/(3*h) x(l1,1) x(l1-1,1) z(i)];
            Spline_0th.coefs(i,1:order) = d;
        case 6
            d  = [(x(l1+4,1)-x(l1,1))/(5*h) x(l1,1)    x(l1-1,1) ...
                   x(l1-2,1)                x(l1-3,1)  z(i)];
            Spline_0th.coefs(i,1:order) = d;
    end
end
%
h  = t(pieces+1)-t(pieces);
l1 = n*pieces;
%
%... Natural Cubic Spline
if (spline_type == 1)
    switch order
        case 4
            d  = [-x(l1,1)/(3*h) x(l1,1) x(l1-1,1) z(pieces)];
            Spline_0th.coefs(pieces,1:order) = d;
        case 6
            d  = [-x(l1,1)/(5*h) x(l1,1)    x(l1-1,1) ...
                   x(l1-2,1)     x(l1-3,1)  z(pieces)];
            Spline_0th.coefs(pieces,1:order) = d;
    end
%
%... Periodic Cubic Spline
elseif (spline_type == 2)
    switch order
        case 4
            d  = [(x(2,1)-x(l1,1))/(3*h) x(l1,1) x(l1-1,1) z(pieces)];
            Spline_0th.coefs(pieces,1:order) = d;
        case 6
            d  = [(x(4,1)-x(l1,1))/(5*h) x(l1,1)    x(l1-1,1) ...
                   x(l1-2,1)             x(l1-3,1)  z(pieces)];
            Spline_0th.coefs(pieces,1:order) = d;
    end
end
%
%% ... Finish DSM_spline function
end

