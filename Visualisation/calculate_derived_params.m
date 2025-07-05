%Name: Alex
%Date: 2/8/24
%Description: Calculates a number of useful values based on the parameters
%of the aircraft

function [derived_parameters, derived_param_names] = calculate_derived_params(...
    parameters, param_names, motor_table, battery_table, airfoil_table)

    horizontal_stabiliser_thickness = ...
        parameters(find(strcmp(param_names, "horizontal_stabiliser_thickness")));
    vertical_stabiliser_thickness = ...
        parameters(find(strcmp(param_names, "vertical_stabiliser_thickness")));
    wing_root_thickness = ...
        parameters(find(strcmp(param_names, "wing_root_thickness")));
    wing_scale = ...
        parameters(find(strcmp(param_names, "wing_scale")));
    wing_length = ...
        parameters(find(strcmp(param_names, "wing_length")));
    wing_sweep = parameters(find(strcmp(param_names, "wing_sweep")));
    horizontal_stabiliser_width = ...
        parameters(find(strcmp(param_names, "horizontal_stabiliser_width")));
    
    motor_index = parameters(find(strcmp(param_names, "motor_index")));
    battery_index = parameters(find(strcmp(param_names, "battery_index")));

    motor_width = motor_table{motor_index, 2};

    motor_mass = motor_table{motor_index, 7};

    battery_mass = battery_table{battery_index, 3};

    tailcone_length = parameters(find(strcmp(param_names, "tailcone_length")));
    empennage_length = ...
        parameters(find(strcmp(param_names, "empennage_length")));
    fuselage_height = ...
        parameters(find(strcmp(param_names, "fuselage_height")));
    fuselage_length = ...
        parameters(find(strcmp(param_names, "fuselage_length")));
    fuselage_param_1 = ...
        parameters(find(strcmp(param_names, "fuselage_param_1")));
    fuselage_param_2 = ...
        parameters(find(strcmp(param_names, "fuselage_param_2")));
    wing_vertical_position = ...
        parameters(find(strcmp(param_names, "wing_vertical_position")));
    wing_horizontal_position = ...
        parameters(find(strcmp(param_names, "wing_horizontal_position")));
    nosecone_length = ...
        parameters(find(strcmp(param_names, "nosecone_length")));
    nosecone_tip_scale = ...
        parameters(find(strcmp(param_names, "nosecone_tip_scale")));
    nosecone_offset = ...
        parameters(find(strcmp(param_names, "nosecone_offset")));
    mesh_resolution = ...
        parameters(find(strcmp(param_names, "mesh_resolution")));
    

    %Width of the rods that the stabilisers on the empennage connect to
    empennage_rod_width = max([horizontal_stabiliser_thickness, vertical_stabiliser_thickness]) + 0.02;


    wing_airfoil_index = parameters(find(strcmp(param_names, "wing_airfoil_index")));

    wing_airfoil_max_camber = airfoil_table{wing_airfoil_index, 2};
    wing_airfoil_max_camber_pos = airfoil_table{wing_airfoil_index, 3};
    wing_airfoil_max_thickness = airfoil_table{wing_airfoil_index, 4};

    wing_airfoil_points = generate_NACA_airfoil(wing_airfoil_max_camber, ...
        wing_airfoil_max_camber_pos, wing_airfoil_max_thickness, ...
        wing_root_thickness, 20);

    %Gets initial chord of wing
    wing_root_chord = max(wing_airfoil_points(:, 1)) - min(wing_airfoil_points(:, 1));

    
    horizontal_stabiliser_airfoil_index = ...
        parameters(find(strcmp(param_names, "horizontal_stabiliser_airfoil_index")));

    horizontal_stabiliser_airfoil_max_camber = airfoil_table{horizontal_stabiliser_airfoil_index, 2};
    horizontal_stabiliser_airfoil_max_camber_pos = airfoil_table{horizontal_stabiliser_airfoil_index, 3};
    horizontal_stabiliser_airfoil_max_thickness = airfoil_table{horizontal_stabiliser_airfoil_index, 4};

    %Generates the airfoil based on parameters
    horizontal_stabiliser_points = generate_NACA_airfoil(horizontal_stabiliser_airfoil_max_camber, ...
        horizontal_stabiliser_airfoil_max_camber_pos, horizontal_stabiliser_airfoil_max_thickness, ...
        horizontal_stabiliser_thickness, 20);

    %Finds chord of horizontal stabiliser
    horizontal_stabiliser_chord = max(horizontal_stabiliser_points(:, 1)) - min(horizontal_stabiliser_points(:, 1));


    %Repeats for the vertical stabiliser
    vertical_stabiliser_airfoil_index = parameters(find(strcmp(param_names, "vertical_stabiliser_airfoil_index")));

    vertical_stabiliser_airfoil_max_camber = airfoil_table{vertical_stabiliser_airfoil_index, 2};
    vertical_stabiliser_airfoil_max_camber_pos = airfoil_table{vertical_stabiliser_airfoil_index, 3};
    vertical_stabiliser_airfoil_max_thickness = airfoil_table{vertical_stabiliser_airfoil_index, 4};

    vertical_stabiliser_points = generate_NACA_airfoil(vertical_stabiliser_airfoil_max_camber, ...
        vertical_stabiliser_airfoil_max_camber_pos, vertical_stabiliser_airfoil_max_thickness, ...
        vertical_stabiliser_thickness, mesh_resolution);

    vertical_stabiliser_chord = max(vertical_stabiliser_points(:, 1)) - min(vertical_stabiliser_points(:, 1));

    %Finds the length of the rods that the stabilisers attach to
    empennage_rod_length = max([horizontal_stabiliser_chord, vertical_stabiliser_chord]);


    %Finds the position of the wing along the z-axis
    wing_zPos = (wing_vertical_position - 0.5) * (fuselage_height-wing_root_thickness);

        %Finds the position of the wing along the y axis
    P1 = [0,fuselage_height*0.5];
    P2 = [fuselage_param_1,fuselage_height*0.5];
    P3 = [fuselage_param_2,-fuselage_height*0.5];
    P4 = [0,-fuselage_height*0.5];

    bez_x_upper = @(t) (1-t).^3 * P1(2) + t.*P2(2).*(3*(1-t).^2) + P3(2)*(3*(1-t).*t.^2) + P4(2).*t.^3 + wing_root_thickness*0.5 - wing_zPos;
    bez_x_lower= @(t) (1-t).^3 * P1(2) + t.*P2(2).*(3*(1-t).^2) + P3(2)*(3*(1-t).*t.^2) + P4(2).*t.^3 + wing_root_thickness*0.5 - wing_zPos;

    t_value_upper = fsolve(bez_x_upper, 0.5, optimoptions ('fsolve', 'Display', 'none'));
    t_value_lower = fsolve(bez_x_lower, 0.5, optimoptions ('fsolve', 'Display', 'none'));


    wing_yPos_upper = (1-t_value_upper).^3 * P1(1) + t_value_upper.*(3*(1-t_value_upper).^2)*P2(1)...
        + (3*(1-t_value_upper).*t_value_upper.^2)*P3(1) + t_value_upper.^3 * P4(1);

    wing_yPos_lower = (1-t_value_lower).^3 * P1(1) + t_value_lower.*(3*(1-t_value_lower).^2)*P2(1)...
        + (3*(1-t_value_lower).*t_value_lower.^2)*P3(1) + t_value_lower.^3 * P4(1);
    
    wing_yPos = min([wing_yPos_lower, wing_yPos_upper]);

    horizontal_stabiliser_full_width = horizontal_stabiliser_width+2*wing_yPos-horizontal_stabiliser_thickness;


    %Length of the first segment of the wing
    wing_base_length = (horizontal_stabiliser_full_width)*0.5-motor_width*0.5-wing_yPos;


    %Ratio between chord at the beginning of the wing base and the chord at
    %the end
    wing_base_scale = 1-((1-wing_scale)*(wing_base_length/wing_length));
    

    %Amount of sweep in the base section of the wing
    wing_base_sweep = (wing_base_length/wing_length)*wing_sweep;
    
    %Finds the position of the wing along the x-axis
    wing_xPos = (fuselage_length-wing_root_chord)*wing_horizontal_position+wing_root_chord*0.5;

    %Finds the ratio of the size of the wing from one side of the motor pod
    %to the other
    wing_motor_scale = 1-(1-wing_scale/wing_base_scale)*(motor_width/(wing_length-wing_base_length));
    
    %Finds the wing sweep across the motor
    wing_motor_sweep = (motor_width/wing_length)*wing_motor_scale*wing_sweep;

    
    motor_pod_length = max([wing_root_chord*wing_base_scale, ...
        wing_root_chord*wing_base_scale*wing_motor_scale+wing_motor_sweep]);

    %Finds the length of the tip of the wing
    wing_tip_length = wing_length-wing_base_length-motor_width;

    %Finds the ratio of the size of the wing tip at the beginning to the
    %size of the tip at the end
    wing_tip_scale = wing_scale/(wing_base_scale*wing_motor_scale);

    %Finds the sweep of the wing tip
    wing_tip_sweep = (wing_tip_length/wing_length)*wing_sweep+...
        wing_root_chord*(wing_base_scale*wing_motor_scale-wing_base_scale*wing_motor_scale*wing_tip_scale);

    empennage_support_length = wing_xPos - wing_base_sweep...
        - 0.5*wing_root_chord + tailcone_length + empennage_length;

    nosecone_solid_height = nosecone_tip_scale*fuselage_height;

    %Finds the curve of the nosecone
    Q2xPos = (nosecone_solid_height*nosecone_length)/(fuselage_height-0.5*nosecone_solid_height);
    nosecone_hollow_length = nosecone_length - 0.5*Q2xPos;

    %Find slope of tangent to the point 
    upperSlope = ((nosecone_solid_height-fuselage_height)*0.5+nosecone_offset)...
    /(nosecone_hollow_length);

    Q2 = [Q2xPos+nosecone_hollow_length, Q2xPos*upperSlope+nosecone_solid_height*0.5+nosecone_offset*0.5];

    %Defines the start and end of the belzer curve as the points at the end of
    %the hollow section
    Q1 = [nosecone_hollow_length,-nosecone_solid_height*0.5+nosecone_offset];
    Q3 = [nosecone_hollow_length,nosecone_solid_height*0.5+nosecone_offset];

    num_t = ceil(mesh_resolution*Q2xPos);

    t_star = linspace(0,0.5,num_t);

    [nosecone_cap_profile, nosecone_cap_x] = bezier_curve_profile(t_star,Q1,Q2,Q3);

    nosecone_cap_x = nosecone_cap_x(end-1)+10^-5;

    nosecone_cap_upper = nosecone_cap_profile(end-1, 1);
    nosecone_cap_lower = nosecone_cap_profile(end-1, 2);
    
    derived_param_names = ...
        ["empennage_rod_width", "wing_root_chord", "wing_base_length", "wing_base_scale",...
        "wing_base_sweep", "motor_pod_length", "motor_width", "motor_mass", ...
        "battery_mass", "empennage_support_length", "empennage_rod_length", ...
        "horizontal_stabiliser_full_width", "horizontal_stabiliser_chord" ...
        "vertical_stabiliser_chord", "wing_tip_length", ...
        "wing_airfoil_max_camber", "wing_airfoil_max_camber_pos",...
        "wing_airfoil_max_thickness", "wing_tip_sweep", "wing_tip_scale", "wing_motor_scale",...
        "wing_motor_sweep", "wing_xPos", "wing_yPos", "wing_zPos", ...
        "horizontal_stabiliser_airfoil_max_camber", "horizontal_stabiliser_airfoil_max_camber_pos",...
        "horizontal_stabiliser_airfoil_max_thickness", "vertical_stabiliser_airfoil_max_camber", ...
        "vertical_stabiliser_airfoil_max_camber_pos", "vertical_stabiliser_airfoil_max_thickness", ...
        "nosecone_cap_upper", "nosecone_cap_lower", "nosecone_cap_x"];

    derived_parameters = ...
        [empennage_rod_width, wing_root_chord, wing_base_length, wing_base_scale, ...
            wing_base_sweep, motor_pod_length, motor_width, motor_mass, ...
            battery_mass, empennage_support_length, empennage_rod_length, horizontal_stabiliser_full_width, ...
            horizontal_stabiliser_chord, vertical_stabiliser_chord, ...
            wing_tip_length, wing_airfoil_max_camber,...
            wing_airfoil_max_camber_pos, wing_airfoil_max_thickness, wing_tip_sweep, ...
            wing_tip_scale, wing_motor_scale, wing_motor_sweep, wing_xPos,...
            wing_yPos, wing_zPos, horizontal_stabiliser_airfoil_max_camber, ...
            horizontal_stabiliser_airfoil_max_camber_pos, horizontal_stabiliser_airfoil_max_thickness,...
            vertical_stabiliser_airfoil_max_camber, vertical_stabiliser_airfoil_max_camber_pos, ...
            vertical_stabiliser_airfoil_max_thickness, nosecone_cap_upper, nosecone_cap_lower, nosecone_cap_x];

end