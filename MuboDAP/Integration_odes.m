function [t,y] = Integration_odes(y_init,solver,tspan)
%
% INTEGRATION: Function that invokes the numerical integrator
%
% Jorge Ambrosio
% Version 1.0     May 12, 2018
% Version 2.0     Aug 21, 2018
%
%%   Access the global variables
%
global tend w tstart count
%
%... Create waitbar
w       = waitbar(0,'Dynamic Analysis Progress');
tstart    = tspan(1);
tend    = tspan(end);
%
count = 0;
%%... Integration of the equations of motion
[t, y]  = feval(solver,@FuncEval,tspan,y_init);
%
%... Close the waitbar
close(w);
%
%... Finish function Integration_odes
end
