function [Penalty ] = Virtue(Cam, Flag)
%Virtue
%
%Summary: This function returns null values while the value of an angle is
%         searched inside a feasible range.
%
%Input:   Cam     - Structure with the values required for function
%         Flag    - Position (=1) or Jacobian (=2)
%
%Output:  Penalty - value for the penalty
%
%%
%
switch Flag
    case 1
        if     Cam.angle < Cam.LoAngle
            Penalty = exp(Cam.LoAngle-Cam.angle)-1;
        elseif Cam.angle > Cam.HiAngle
            Penalty = exp(Cam.angle-Cam.HiAngle)-1;
        else
            Penalty = 0;
        end
    case 2
        if     Cam.angle < Cam.LoAngle
            Penalty =-exp(Cam.LoAngle-Cam.angle);
        elseif Cam.angle > Cam.HiAngle
            Penalty = exp(Cam.angle-Cam.HiAngle);
        else
            Penalty = 0;
        end
end
%
%%
%... Finalize function Virtue
end