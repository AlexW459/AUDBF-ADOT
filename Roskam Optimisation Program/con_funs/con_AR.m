function AR = con_AR(plane)


    AR = [plane.wing.aspect_ratio_max - plane.wing.aspect_ratio;
          plane.wing.aspect_ratio - plane.wing.aspect_ratio_min];

    


end