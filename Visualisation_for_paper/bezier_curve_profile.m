%Name:   Alex, Isaac
%Date:   22/07/2024
%Purpose: Generate [lower, upper] values for a Bezier curve side profile.

function [profile, x_sample] = bezier_curve_profile(t_star,P1,P2,P3)
    x_sample = [];
    lower = [];
    upper = [];

    for t_index = 1:size(t_star,2)

        t = t_star(t_index);

        %Point 1 is assumed to be above point 3

        %Points on Bezier curve are calculated
        point_x_pos = (1-t).^2*P1(1)+2*(1-t).*t*P2(1)+t.^2*P3(1);
        upper_y_pos = (1-t).^2*P1(2)+2*(1-t).*t*P2(2)+t.^2*P3(2);
        %t is replaced with 1-t to find points on opposite side of curve
        lower_y_pos = (t).^2*P1(2)+2*(t).*(1-t)*P2(2)+(1-t).^2*P3(2);
        
        x_sample = [x_sample, point_x_pos];
        lower = [lower; lower_y_pos];
        upper = [upper; upper_y_pos];

        
    end
    profile = [lower, upper];

end