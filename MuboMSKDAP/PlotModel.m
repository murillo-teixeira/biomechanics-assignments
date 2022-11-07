function PlotModel(q, PltMuscles)
%
%Summary: This function plots the model under analysis. It is not generic.
%         It is using the information from the revolute joints to compute
%         the connections between bodies. If other joints exist, the model
%         will not be fully plotted.
%
%Input:   q        - Matrix with the history of position
%
%Output:  No output specified 
%
%... Access memory
global Ntime NBody Body Jnt

% Goes through all time steps
for k = 1 : Ntime

    % Animates the stick figure man
    % Transfer data to working variables
    for j = 1 : NBody
        % Indices for position and orientation
        i1 = 3 * (j - 1) + 1;
        i3 = i1 + 2;
        
        % Updates the body data
        Body(j).r = q(i1 : i1 + 1, k);
        Body(j).theta = q(i3, k);
        
        % Updates the matrices A and B
        costheta = cos(Body(j).theta);
        sintheta = sin(Body(j).theta);
        Body(j).A = [costheta, -sintheta;
            sintheta, costheta];
        Body(j).B = [-sintheta, -costheta;
            costheta, -sintheta];
    end
    
    % Goes through the revolute joints and plots the segments that go from
    % the com to the joint
    for j = 1 : Jnt.NRevolute
        
        % Define local variables
        Bodyi = Jnt.Revolute(j).i;
        Bodyj = Jnt.Revolute(j).j;
        spPi = Jnt.Revolute(j).spPi;
        spPj = Jnt.Revolute(j).spPj;
        
        % Defines the global coordinates of the point i
        Pi = Body(Bodyi).r + Body(Bodyi).A * spPi;
        Pj = Body(Bodyj).r + Body(Bodyj).A * spPj;
        
        % Plots the segments
        plot([Body(Bodyi).r(1), Pi(1)], [Body(Bodyi).r(2), Pi(2)], 'b');
        hold on;
        plot([Body(Bodyj).r(1), Pj(1)], [Body(Bodyj).r(2), Pj(2)], 'b');
        hold on;
        
    end
    
    if (nargin > 1 && PltMuscles == 1)
        % Goes through the muscles and plots the muscles
        for m = 1 : Jnt.NMuscles
            
            % Number of muscle segments
            NSegments = length(Jnt.Muscle(m).spBody) - 1;
            
            % Goes through all segments and plots the segment
            for s = 1 : NSegments
                
                % Define local variables
                Bodyi = Jnt.Muscle(m).spBody(s);
                spPi   = Jnt.Muscle(m).spP(s,:)';
                Bodyj = Jnt.Muscle(m).spBody(s+1);
                spPj   = Jnt.Muscle(m).spP(s+1, :)';
                
                % Defines the global coordinates of the attachment points
                Pi = Body(Bodyi).r + Body(Bodyi).A * spPi;
                Pj = Body(Bodyj).r + Body(Bodyj).A * spPj;
                
                try
                    if (Jnt.Muscle(m).Activations(k) > 0.01)
                        plot([Pi(1) Pj(1)], [Pi(2), Pj(2)], '--r');
                    else
                        plot([Pi(1) Pj(1)], [Pi(2), Pj(2)], '--k');
                    end
                catch
                    plot([Pi(1) Pj(1)], [Pi(2), Pj(2)], '--k');
                end
            end
            
            % End of the loop that goes through all muscles
        end
    end
    
    axis([-1 2 0 2]);
    pause(0.001);
    hold off;
%     pause
    % End of function
end