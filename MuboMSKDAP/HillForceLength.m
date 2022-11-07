function Fl = HillForceLength(lM, F0, L0)
%
%Summary: This function defines the forces resulting from the Hill
%         force-length relationship (taken from the work of Silva et al.
%         2003).
%
%Input:   lM       - Muscle fiber length
%         F0       - Maximum isometric muscle force
%         L0       - Optimum muscle fiber length
%
%Output:  Fl       - Force-length relationship force
%

a = (-(9/4) * (lM / L0 - 19/20));
            
Fl = F0 * exp(-(a^4 - (1/4) * a^2));

% End of function
end