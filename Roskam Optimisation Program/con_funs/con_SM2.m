%Name:    Isaac Nakone
%Date:    11/12/2023
%Purpose: This is a function which takes in an 
%         aircraft and outputs the clearance 
%         between the minimum static margin 
%         and the actual static margin for M2.

%Description from constraint set:
%   Static margin for mission 2 versus minimum required (SM2min)


function SM2 = con_SM2(plane)

    calc_COG2(plane);

    calc_COG3(plane);
    
    plane.horz_stabiliser.aspect_ratio = plane.horz_stabiliser.span...
                                        /plane.horz_stabiliser.chord_root;
 
  


    plane.wing.MAC = (2/3)*plane.wing.chord_root*...
                    (1+plane.wing.taper_ratio+plane.wing.taper_ratio^2)...
                    /(1+plane.wing.taper_ratio);
    
    plane.wing.aerodynamic_center2 = plane.wing.position +...
                                     0.25*plane.wing.MAC;                                                   %Aerodynamic center of wing
    plane.horz_stabiliser.aerodynamic_center2 =...
        plane.wing.span/plane.misc_properties.aspect_ratio - ...
        0.75*plane.horz_stabiliser.chord_root;                                               %Aerodynamic center of the tail

    
    t1 = 1-2*plane.horz_stabiliser.coeff_liftA/pi/...
         plane.wing.aspect_ratio;
    t2 = plane.horz_stabiliser.area/plane.wing.area;
    t3 = plane.horz_stabiliser.coeff_liftA*t1*t2;
    plane.aerodynamic_center2 = (plane.wing.aerodynamic_center2+...
        (plane.horz_stabiliser.aerodynamic_center2* t3/plane.wing.coeff_liftA))/...
        (1+ t3/plane.wing.coeff_liftA);

  

    

    
    plane.static_margin2 =...
          (plane.aerodynamic_center2-plane.COG2)/...
          plane.wing.MAC;  
    
    SM2min = plane.static_margin2 - ...
             plane.static_margin_min_req;

    SM2max = plane.static_margin_max_req - ...
             plane.static_margin2;


    SM2 = [SM2min;
           SM2max];
end