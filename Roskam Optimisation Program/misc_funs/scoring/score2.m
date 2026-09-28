%Name:    Isaac Nakone, Alec Vitalievich
%Date:    08/09/2024
%Purpose: This calculates the score for mission 2, as well as updating the
%         relevant parameters of the aircraft.

function score2(plane)
    
    %Archived: calculate the score for mission 2 and store in the plane variable.
    plane.score.M2 = 1.0 + plane.payload.mass2...
                     /(3*plane.cruise_properties.time2)/...
                     plane.score.baseline2;

    % For Alec: Write the new score formula here, try testing them out in dummy form
    % before we implement the new aircraft class and update teh formula
    % then
   

end
