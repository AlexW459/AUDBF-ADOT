
function calc_payload_mass3(plane)

    plane.passengers.mass = plane.passengers.total*...
        plane.passengers.doll_mass;   

    plane.payload.mass3 = plane.passengers.mass+plane.trays.mass;  

end