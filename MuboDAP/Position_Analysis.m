function [ q,Jac ] = Position_Analysis(time,q)
%Position_Analysis
%
%Summary: This function controls the kinematic position analysis. The
%         method used is the Newton-Raphson.
%
%Input:   time - Time instant in which the positions are evaluated
%         q    - Estimate for the positions
%
%Output:  q    - Corrected positions
%         Jac  - Jacobian matrix
%
%Shared:  tolerance - Tolerance for the convergence of the NR method
%         itermax1  - Maximum number of iterations allowed plus 1
%
%%
%... Access the global memory
global parameters Flag
%
%... Set the analysis flags
Flag.Transfer = 1;
Flag.General  = 1;
Flag.Position = 1;
Flag.Jacobian = 1;
%
%... Solve the nonlinear system of equations using Newton-Raphson
for iter = 1:parameters.NRMaxIter
%
%... Evaluate constraint equations and Jacobian matrix
    [Phi,Jac,~,~] = KinemEval(time,q,[]);
%
%... Evaluate the position correction
    dq = Jac\Phi;
%
%... Correct positions
    q = q-dq;
%
%... Check if solution is obtained
    if max(abs(dq))<parameters.NRTolerance
        break
    end
%
%... Check if maximum number of iterations is reached and warn
    NRMaxIter = parameters.NRMaxIter;
    if iter == parameters.NRMaxIter
        string = ['Max iterations of NR (' num2str(NRMaxIter) ') reached'];
        disp(string)
        string = ['@ Time = ' num2str(time)];
        disp(string)
    end
end
%
%... Reset analysis flags
Flag.Transfer = 0;
Flag.Position = 0;
Flag.General  = 0;
Flag.Jacobian = 0;
%%
%... Finalize function Position_Analysis
end