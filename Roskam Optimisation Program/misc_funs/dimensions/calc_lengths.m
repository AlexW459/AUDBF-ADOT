%Name:    Isaac Nakone, Denis Vasilyev
%Date:    12/12/2023
%Purpose: This is a function which takes in an 
%         aircraft and setsd the lengths of different parts of the fuselage




function calc_lengths(plane)

    plane.msc.width = max(plane.fuselage.width ...
                      -2*plane.fuselage.object_spacing,...
                      3*plane.units.in2mm);  
    plane.msc.length  = max(plane.msc.mass...
                        /plane.msc.width...
                        /plane.crew.doll_length...
                        /plane.msc.density,...
                        3*plane.units.in2mm);               % length of medical supply cabinet              [m].

    
    plane.fuselage.height  = plane.fuselage.wall_thickness+...
                             plane.trays.thickness+...
                             plane.crew.doll_length+...
                             3*plane.fuselage.object_spacing;          % fuselage height for mission 2 and 3              [m].
    plane.nose.length = plane.battery.length+0.05;                    % length of nose for mission 2                     [m].
    plane.fuselage.tail_length = plane.fuselage.height/...
                        tand(plane.vert_stabiliser.angle);                  % length of tail section for mission 2             [m].


    
                           %length of the medical compartment for mission 2  [m].

                           
    plane.passengers.length = (plane.passengers.doll_width+....             %Length of passenger compartment for mission3 
        plane.fuselage.object_spacing)*plane.passengers.total/...
        plane.passengers.nwidth+plane.fuselage.object_spacing;  

    plane.medical_bay.length = plane.passengers.doll_length +...
                               2*plane.fuselage.object_spacing; 
    
    plane.cabin.length = plane.passengers.doll_width + ...                     %Length of cabin
        2*plane.fuselage.object_spacing;    
    
    
    plane.fuselage.length2 = plane.cabin.length + ...                       %Length of fuselage required for mission 2
        plane.medical_bay.length + plane.msc.length + ...
        plane.nose.length + plane.fuselage.tail_length +...
        2*plane.fuselage.object_spacing;
    
    
    plane.fuselage.length3 = plane.passengers.length + ...                  %Length of fuselage required for mission 3
        plane.fuselage.wall_thickness+plane.cabin.length + ...
        plane.nose.length + plane.fuselage.tail_length;

    plane.fuselage.length = max(plane.fuselage.length2,...                  %The length of the fuselage to be built is the greatest of the requirenmenets for missions 2 and 3
        plane.fuselage.length3);
    
    if(plane.fuselage.length < plane.wing.chord_root*1.1)                     % if fuelsage length is less than 1.1 times wing root chord, set fuselage length to 1.1 times the wing root chord.
        plane.fuselage.length = plane.wing.chord_root*1.1; %Is this correct???
    end


     plane.fuselage.form_factor = 1+(60)/((plane.fuselage.length/...
                                plane.fuselage.width)^3)+(plane.fuselage.length/...
                                plane.fuselage.width)/(400);

     

end