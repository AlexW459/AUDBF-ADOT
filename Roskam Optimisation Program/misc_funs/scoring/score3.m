%Name:    Denis Vasilyev, Alec Vitalievich
%Date:    08/09/2024
%Purpose: This calculates the score for mission 3, as well as updating the
%         relevant parameters of the aircraft.

function score3(plane)
    
    %Archive: calculate the score for mission 3 and store in the plane variable.

    plane.cruise_properties.laps3 = 5*60/plane.cruise_properties.time3;
 
    plane.score.M3 = 2.0 + plane.cruise_properties.laps3*...
        plane.passengers.total/plane.battery.capacity/plane.score.baseline3;

    % For Alec: Write the new score formula here, try testing them out in dummy form
    % before we implement the new aircraft class and update teh formula
    % then
    

end