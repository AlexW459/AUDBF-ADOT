
function calc_payload_mass2(plane)

    
    plane.emt.mass     = 2*plane.crew.mass;                         % mass of the EMT                                  [kg].
    plane.guerney.mass = plane.passengers.doll_length*...
                         plane.passengers.doll_width*...
                         0.01*plane.misc_properties.density_PETG; 

    plane.payload.mass2 = 2*plane.crew.mass + 2*plane.emt.mass + plane.servos.mass + ...
                          + plane.guerney.mass + plane.msc.mass + plane.trays.mass;  

end