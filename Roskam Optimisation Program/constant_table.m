%Name:        Isaac Nakone, Denis Vasilyev, Harry Rowton, Jingya Liu
%Date:        15/12/2023
%Purpose:  This is a class to be used to store the 
%          ADOT constraints.

classdef constant_table

    properties
        %Constants related to the whole plane:
        
        static_margin_min_req = 0.01;
        static_margin_max_req = 0.08;

        num_power_vals = 10;

        %Unit conversions:
        units = struct('in2mm', 0.0254, 'g2kg', 1e-3, 'mm2m', 1e-3,...
                       'h2s', 3600, 'min2s', 60, 'W2mW', 1e3, 'min2h', 0.0167,...
                       'kg2lb', 2.2046, 'W2hp', 0.0013, 'm2ft', 3.2808, 'ft2m', 0.3048);

        %Physical parameters:
        physics = struct('air_density0',1.225, ...
            'air_density30',1.775, 'gravity',9.81, ...
            'kinematic_viscosity', 0.00001798,...
            'gas_constant', 287, 'thermodynamic_ratio', 1.4,...
            'temperature', 12.224 + 273.15);


        %Size of the course that we will fly through.
        %takeoff distance updated to 100 feet for 2025 competition
        course = struct('length', 610, 'takeoff_distance' ,100);

        score = struct('baseline2',0.06,'baseline3',0.2);

        passengers = struct('doll_length', 0.1397, 'doll_width', 0.0428, 'doll_mass',0.015);
        
        fuselage = struct('density', 1230, ...
                           'wall_thickness',0.0008, 'tray_thickness',0.005,...
                            'object_spacing',0.01,...
                            'interference_factor', 1.0);

        wing = struct('density',1230,'sweep_angle25', 0,...
                       'interference_factor', 1.1,'aspect_ratio_max', 18,...
                       'aspect_ratio_min', 3, 'layer_thickness', 0.0008);


        horz_stabiliser = struct( 'interference_factor', 1.04, ...
                     'area_density', 0.1);
        vert_stabiliser = struct( 'interference_factor', 1.04,...
                                 'angle', 60, 'area_density', 0.1);


        misc_properties = struct('load_factor',5,...
                                 'time_safety', 1.0, ...
                                 'propellor_efficiency',0.75,...
                                 'electrical_efficiency',0.95,...
                                 'propulsive_efficiency',0.75*0.95, ...
                                 'density_PETG', 1230.0, ...
                                 'stall_speed_max', 7);

        battery = struct('voltage', 22.2);

        crew = struct('mass',0.035,'doll_length', 0.1397);

        emt = struct('mass',0.035);

        msc = struct('density', 8390);

        nose = struct('angle', 45);

        servos = struct('mass', 0.012, 'number', 8);

        guerney = struct('mass', 0.02);

        patient = struct('mass', 0.015); %Change this a sI'm not sure of patient mass.

        landing_gear = struct('mass', 0.1,'wheel_width',0.025,...
            'wheel_height', 0.080, 'strut_thickness1', 0.003, ...
            'strut_thickness2', 0.001);

        electronics = struct('mass', 0.1);

        trays = struct('thickness', 0.005);
    end


end