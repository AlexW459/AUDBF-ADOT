%Name:    Isaac Nakone
%Date:    11/12/2023
%Purpose: This calculates the score for mission 2, as well as updating the
%         relevant parameters of the aircraft.

function S = score(plane, opt_vars, stab_vars)

    if (nargin == 3)

        plane.link_opt_vars(opt_vars); %Need to do this to update plane object.

        plane.link_stab_vars(stab_vars);

    end

    calcs(plane);

    plane.score.M1 = 1.0; %assume this by default;

    score2(plane);

    score3(plane);

    plane.score.total = plane.score.M1 + plane.score.M2 + plane.score.M3;


    S = plane.score.total;
    
end