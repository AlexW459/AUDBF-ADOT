
function calc_design_mass(plane)

    plane.mass2 = plane.fuselage.empty_mass+plane.crew.mass*2+...
        plane.emt.mass*2+plane.patient.mass+plane.guerney.mass+...
        plane.msc.mass;
    plane.mass3 = plane.fuselage.empty_mass+plane.passengers.mass;
    plane.mass    = max(plane.mass2, plane.mass3);                        % mass of the EMT                                  [kg].                           [kg].


end