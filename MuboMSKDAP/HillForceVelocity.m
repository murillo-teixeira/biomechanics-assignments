function Fld = HillForceVelocity(lMd, F0, Vmax)
%
%Summary: This function defines the forces resulting from the Hill
%         force-velocity relationship (taken from the work of Silva et al.
%         2003).
%
%Input:   lMd      - Muscle velocity of contraction
%         F0       - Maximum isometric muscle force
%         Vmax     - Maximum velocity of contraction
%
%Output:  Fld      - Force-velocity relationship force
%

if ((- Vmax) > lMd)
    Fld = 0;
else
    if ( (lMd <= 0.2 * Vmax) && (lMd >= (-Vmax)))
        Fld = - (F0 / atan(5)) * atan( -5 * (lMd / Vmax)) + F0;
    else
        Fld = (pi() * F0) / (4 * atan(5)) + F0;
    end
end

% End of function
end