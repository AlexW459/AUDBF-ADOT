

function CR = con_CR(plane)
    CR = [plane.motor.power_max*plane.motor.number - plane.cruise_properties.power2;
            plane.motor.power_max*plane.motor.number - plane.cruise_properties.power3];
end