function Fpe = HillPassiveForce(lM, F0, L0)
%
%Summary: This function defines the forces resulting from the passive
%         component of the Hill model (taken from the work of Silva et al.
%         2003).
%
%Input:   lM       - Muscle fiber length
%         F0       - Maximum isometric muscle force
%         L0       - Optimum muscle fiber length
%
%Output:  Fpe       - Passive force
%

% Length boundaries
upbound = 1.63;
lobound = 1;
    

% CASO ORIGINAL
if ((lobound * L0) > lM)
    Fpe = 0;
else
    if ( (lM <= (upbound * L0)) && (lM >= (lobound * L0)))
        Fpe = 8 * (F0 / L0^3) * (lM - (lobound * L0))^3;
    else
        Fpe = 2 * F0;
    end
end

% End of function
end