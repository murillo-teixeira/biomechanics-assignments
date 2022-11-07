function lambda = optimization_dyn(q, qd, qdd, t)
%
%Summary: This function performs an inverse dynamic analysis using an
%         optimization procedure.
%
%Input:   q      - Matrix of positions of the system
%         qd     - Matrix of velocities of the system
%         qdd    - Matrix of accelerations of the system
%         t      - Vector of time frames
%
%Output:  lambda - Vector with Lagrange multipliers of the system
% 
% Jorge Ambrosio
% Version 1.0     May, 2020
%
%% ... Access memory
global Ntime NConstraints parameters
global M Jnt MSKDrivers
global parameters

%% ... Updates muscle kinematics
% Computes muscle kinematics
MuscleKinematics(q, t);
%
%% ... Allocates memory for the output
lambda = zeros(NConstraints, Ntime);
for i = 1 : Jnt.NMuscles
    Jnt.Muscle(i).lM          = zeros(Ntime,1);
    Jnt.Muscle(i).lMd         = zeros(Ntime,1);
    Jnt.Muscle(i).Activations = zeros(Ntime,1);
    Jnt.Muscle(i).ForcesPE      = zeros(Ntime, 1);
    Jnt.Muscle(i).ForcesCE      = zeros(Ntime, 1);
end
%
%% Goes through all steps
%... Create waitbar
w       = waitbar(0,'Dynamic Analysis Progress');
%
for i = 1 : Ntime

    % ... Evaluate vectors and matrices
    [ ~,Jac,~,~] = KinemEval(t(i), q(:,i), qd(:,i));
    
    %% ... Create the force vectors of bodies and nodes
    [g]   = Force(t(i));

    %% ... Define boundary conditions for optimization problem
    [lb, ub, optmusc] = OptimizationBoundaries(t(i));
    %
    %% ... Initial solution
    if (i == 1)
        x0 = rand(NConstraints, 1);
    else
        x0 = lambda(:, i - 1) ./ DesScale; % The division results from the fact that lambda was stored using unscaled design variables
    end
    %
    %% ... Defines the optimization options
    options_optimization = optiset('solver', 'IPOPT',...
        'display','off',...
        'maxtime', 2e4,...%'maxtime', 2e3,...
        'maxiter', 3e3,...
        'maxfeval', 2e4,...
        'derivCheck', 'off'); 
%     options_optimization.solverOpts = ipoptset('linear_solver', 'MUMPS',...
%         'jac_c_constant', 'yes');
    
    %% ... Scaling parameters for the design variables
    DesScale = ones(size(x0));
    DesScale(parameters.drivspan) = 1e-4;
    
    %% ... Builds the optimization problem
    OptProblem = opti('fun', @(x) PhysiolCriter(x, 0),...
        'grad', @(x) PhysiolCriter(x, 1),...
        'eq', (diag(DesScale) * Jac)', (g - M * qdd(:,i) - (Jac(parameters.muscspan,:)' * optmusc.fpe)),...
        'bounds', lb ./ DesScale, ub .* (optmusc.scalJac ./ DesScale),...
        'x0', x0,...
        'options', options_optimization);
    
    %% ... Solves the optimization problem using fmincon. The objective 
    %      function is the square sum of muscle forces plus an additional 
    %      term to penalize joint actuators. The constraints are the 
    %      equations of motion.   
    % Optimization
    [lambda(:,i), fval , exitflag] = solve(OptProblem);
    if (exitflag < 0)
        % Runs the algorithm two more times using a random initial solution
        % to try to find a solution
        BestSolution = lambda(:,i);
        BestFval = fval;
        
        % Variable that control the while loop
        SolutionFound = 0;
        InterIter = 1;
        while (InterIter < 3 && SolutionFound == 0)
            % Defines a novel initial solution
            x0 = rand(NConstraints, 1);
            
            % Rebuilds the optmization problem
            OptProblem = opti('fun', @(x) PhysiolCriter(x, 0),...
                'grad', @(x) PhysiolCriter(x, 1),...
                'eq', (diag(DesScale) * Jac)', (g - M * qdd(:,i) - (Jac(parameters.muscspan,:)' * optmusc.fpe)),...
                'bounds', lb ./ DesScale, ub .* (optmusc.scalJac ./ DesScale),...
                'x0', x0,...
                'options', options_optimization);
            
            % Solves the optimization problem
            [x_r, fval_r , exitflag_r] = solve(OptProblem);
            
            % Checks the conditions to stop the loop
            if (exitflag_r < 0)
                if (fval_r < BestFval)
                    BestFval = fval_r;
                    BestSolution = x_r;
                end
            else
                SolutionFound = 1;
                BestFval = fval_r;
                BestSolution = x_r;
            end
            % Updates the iteration index
            InterIter = InterIter + 1;
        end
        % Saves the best solution
        lambda(:,i) = BestSolution;
        
        if (SolutionFound == 0)
            warning(['The optimization problem did not converge for the time step ', num2str(i), '.\nPlease check if the biomechanical model is properly defined.']);
%             pause
        end
    end
    % Descales the lambda
    lambda(:,i) = lambda(:,i) .* DesScale;
    
    % Updates muscle force data for the report
    for m = 1 : Jnt.NMuscles
        Jnt.Muscle(m).Activations(i) = lambda(parameters.muscspan(m), i) /...
            ((Jnt.Muscle(m).Param(1) * Jnt.Muscle(m).Param(2) / Jnt.Muscle(m).F0));
        Jnt.Muscle(m).ForcesCE(i) = lambda(parameters.muscspan(m), i);
        Jnt.Muscle(m).ForcesPE(i) = optmusc.fpe(m);
        Jnt.Muscle(m).lM(i) = Jnt.Muscle(m).Param(3);
        Jnt.Muscle(m).lMd(i) = Jnt.Muscle(m).Param(4);        
    end
    
    % ... Update the Waitbar
    w = waitbar(i/Ntime,w,['time: ',num2str(t(i),'%10.5f')]);
    
    % End of the loop that goes through all steps
end
%... Close the waitbar
close(w);

%% ... Definition of the nested objective function
    function output = PhysiolCriter(x, grad)
        
        % Initilizes the objective function
        f = 0;
        df = zeros(NConstraints, 1);
        
        % Goes through all muscles
        for k = 1 : Jnt.NMuscles
            
            f = f + 1 * x(parameters.muscspan(k))^2;
            % Updates the gradient of the objective function
            df(parameters.muscspan(k)) = df(parameters.muscspan(k)) +...
                2 * x(parameters.muscspan(k));
            
            % End of the loop that goes through all muscles
        end
        
        % Penalizes the driver actuators. The variable MSKDrivers contains
        % the number of the drivers that are associated with the degrees of
        % freedom that are now driven by muscles
        f = f + 1 * (x(parameters.drivspan(MSKDrivers))' * x(parameters.drivspan(MSKDrivers)));
        df(parameters.drivspan(MSKDrivers)) = df(parameters.drivspan(MSKDrivers)) +...
            2 * 1 * x(parameters.drivspan(MSKDrivers));

        % Returns the output depending on the value of grad
        if (grad == 0)
            output = f;
        else
            output = df';
        end

        % End of the nested function
    end    

% End of the dynamic analysis
end