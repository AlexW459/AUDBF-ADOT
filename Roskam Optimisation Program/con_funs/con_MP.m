
function MP = con_MP(plane)

    

    TOP23 = (-4.9+sqrt(24.01+0.036*plane.course.takeoff_distance))/(0.018); %MAGIC NUMBERS!!
    
    

    plane.course.take_off_power = (plane.mass*plane.units.kg2lb)/...
        ((plane.units.W2hp*TOP23*(plane.physics.air_density30/...
        plane.physics.air_density0)*plane.wing.coeff_lift_max)...                      %required power to take off in the takeoff distance, this is used in motor selection
           /((plane.mass*plane.units.kg2lb)/(plane.wing.area*plane.units.m2ft^2)));

    MP = plane.motor.number*plane.motor.power_max...
         - plane.course.take_off_power;   %Changed this the other way around (correct as of 12/11/2023).


end