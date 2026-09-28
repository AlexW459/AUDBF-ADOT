function WP = con_WP( plane)

    WP = [plane.wing.position - plane.cabin.length;
          (plane.fuselage.length-plane.wing.chord_root) - ...
          plane.wing.position];


end