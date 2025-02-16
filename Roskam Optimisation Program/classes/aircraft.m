%Name:        Isaac Nakone, Denis Vasilyev, Harry Rowton, Jingya Liu
%Date:        15/12/2023
%Description: This is a class definition for an
%             aircraft to be used in the ADOT code.
%             Note that unknown variables are set to
%             NaN to begin with to ensure that we are
%             not using unspecified values in the maths.

classdef aircraft < handle &...  %Inherits handle properties.
        matlab.mixin.SetGet &... %Inherits setters and getters.
        matlab.mixin.Copyable    %Is copyable.
    properties 

        %Variables related to the whole plane:
        static_margin2 = NaN;
        static_margin3 = NaN;

        static_margin_min_req = NaN;
        static_margin_max_req = NaN;

        num_power_vals = NaN;

        COG2 = NaN;
        COG3 = NaN;

        mass2 = NaN;
        mass3 = NaN;

        mass = NaN;

        aerodynamic_center2 = NaN;
        aerodynamic_center3 = NaN;


        %Unit conversions:
        units = struct('in2mm', NaN, 'g2kg', NaN, 'mm2m', NaN,...
                       'h2s', NaN, 'min2s', NaN, 'W2mW', NaN, 'min2h', NaN,...
                       'kg2lb', NaN, 'W2hp', NaN, 'm2ft', NaN, 'ft2m', NaN);

        %Physical parameters:
        physics = struct('air_density0',NaN, ...
            'air_density30',NaN, 'gravity',NaN, ...
            'kinematic_viscosity', NaN,...
            'gas_constant', NaN, 'thermodynamic_ratio', NaN,...
            'temperature', NaN);


        %Size of the course that we will fly through.
        course = struct('length', NaN, 'takeoff_distance' ,NaN);

        score = struct('M1', NaN, 'M2', NaN, 'M3', NaN, 'baseline2',NaN,'baseline3',NaN,...
                        'total', NaN);
    
        %2024 competition
        %passengers = struct('total', NaN, 'nwidth',NaN, 'length',NaN,...
        %                    'doll_length', NaN, 'doll_width', NaN, 'doll_mass',NaN,...
        %                    'mass',NaN);
        
        fuselage = struct('width',NaN, 'length', NaN, 'height', NaN,...
                           'form_factor',NaN,'mass',NaN, 'density', NaN, ...
                           'wall_thickness',NaN, 'tray_thickness',NaN,...
                            'object_spacing',NaN,...
                            'interference_factor', NaN, ...
                            'min_width2',NaN, 'min_width3', NaN, 'reynolds_num', NaN);

        wing = struct('chord_root',NaN, 'chord_tip',...
                       NaN, 'span', NaN, 'position',NaN, ...
                       'coeff_lift_max', NaN, 'taper_ratio', NaN,...
                       'area_density',NaN,'mass',NaN, 'sweep_angle25', NaN,...
                       'interference_factor', NaN, 'chord_thickness', NaN,...
                       'aerodynamic_center2',NaN,...
                       'aerodynamic_center3',NaN,...
                       'aspect_ratio', NaN, ...
                       'coeff_drag_data', [], ...
                       'coeff_lift0',NaN, ...
                       'coeff_liftA',NaN, ...
                       'wetted_area', NaN, ...
                       'aspect_ratio_max', NaN,...
                       'aspect_ratio_min', NaN, ...
                       'layer_thickness', NaN);


        horz_stabiliser = struct('chord_root',NaN, 'chord_tip', ...
                                 NaN, 'span',NaN, ...
                                 'interference_factor', NaN, ...
                                 'mass', NaN, 'area_density', NaN,...
                                 'coeff_drag2', NaN, 'coeff_drag3', NaN);
        vert_stabiliser = struct('chord_root',NaN, 'chord_tip', ...
                                 NaN, 'span',NaN, ...
                                 'interference_factor', NaN,...
                                 'angle', NaN, 'mass', NaN, 'area_density', NaN...
                                 ,'coeff_drag2', NaN, 'coeff_drag3', NaN);


        cruise_properties = struct( 'capacity', NaN, 'laps3', ...
                                    NaN, 'time2', NaN, 'coeff_lift', NaN, 'speed2', NaN, ...
                                     'speed3', NaN, 'acceleration2', NaN, 'acceleration3', NaN);

        turning_properties = struct('speed2',NaN, 'speed3' ,NaN, 'power', NaN, 'capacity', NaN, 'laps3', NaN, 'coeff_lift', NaN);

        misc_properties = struct( ...
                                 'load_factor',NaN, 'aspect_ratio',NaN, ...
                                 'time_safety', NaN, ...
                                 'propellor_efficiency',NaN,...
                                 'electrical_efficiency',NaN,...
                                 'propulsive_efficiency',NaN, ...
                                 'density_PETG', NaN,...
                                 'stall_speed_max', NaN);

        battery = struct('capacity', NaN, 'voltage', NaN, 'mass',NaN,...
                         'height', NaN, 'length', NaN, 'required_capacity2', ...
                         NaN, 'required_capacity3', NaN);

        motor = struct('number', NaN, 'power_max', NaN, 'mass', NaN);
        %2024 competition
        %payload = struct('mass2',NaN, 'mass3',NaN);
        fuel_tank = struct('mass', NaN, 'location', NaN);
        
        %2024 competition
        %crew = struct('mass',NaN,'doll_length', NaN);

        %emt = struct('mass',NaN);

        %msc = struct('mass', NaN, 'density', NaN);

        nose = struct('length', NaN, 'angle', NaN);

        servos = struct('mass', NaN, 'number', NaN);
        
        %2024 competition
        %guerney = struct('mass', NaN);

        %medical_bay = struct('length', NaN);

        %patient = struct('mass', NaN); %Change this a sI'm not sure of patient mass.
        
        x1_prototype = struct('mass', NaN, 'location', NaN);
        
        cabin = struct('length', NaN);

        landing_gear = struct('mass', NaN,'wheel_width',NaN,...
            'wheel_height', NaN, 'strut_thickness1', NaN, ...
            'strut_thickness2', NaN);

        electronics = struct('mass', NaN);

        %trays = struct('thickness', NaN, 'mass', NaN);

        
    end

    methods 

        function link_opt_vars(obj, opt_vars)
            %2024 competition
            %obj.passengers.nwidth            = opt_vars(1);  
            %obj.passengers.total             = opt_vars(2);
            %obj.msc.mass                     = opt_vars(3);
            obj.wing.taper_ratio             = opt_vars(1);
            obj.wing.chord_root              = opt_vars(2);
            obj.fuselage.width               = opt_vars(3);
            obj.wing.span                    = opt_vars(4);
            obj.wing.position                = opt_vars(5);
            obj.horz_stabiliser.chord_root   = opt_vars(6);
            obj.horz_stabiliser.span         = opt_vars(7);
            obj.misc_properties.aspect_ratio = opt_vars(8);
            obj.cruise_properties.speed2     = opt_vars(9);
            obj.cruise_properties.speed3     = opt_vars(10);
            obj.cruise_properties.acceleration2         = opt_vars(11);
            obj.cruise_properties.acceleration3         = opt_vars(12);
            obj.x1_prototype.mass            = opt_vars(13);
            obj.fuel_tank.mass               = opt_vars(14);
            
        end


        function link_stab_vars(obj, stab_vars)
            obj.msc.COG = stab_vars(1);  
        end

        function link_battery(obj, battery_index, batterySpecs)
            obj.battery.capacity = batterySpecs.batteryCapacity_mAh_(battery_index);
            obj.battery.width    = batterySpecs.Bw_mm_(battery_index)*obj.units.mm2m;
            obj.battery.height   = batterySpecs.Bh(battery_index)*obj.units.mm2m;
            obj.battery.length   = batterySpecs.Bl(battery_index)*obj.units.mm2m;
            obj.battery.mass     = batterySpecs.weight_g_(battery_index)*obj.units.g2kg;
        end

        function link_motor(obj, motor_index, motorSpecs)
            obj.motor.power_max = motorSpecs.BatteryPower_W_(motor_index);
            obj.motor.mass = motorSpecs.weight_g_(motor_index)*obj.units.g2kg;
            obj.motor.number = 2;

        end

        function link_airfoils(obj,wingSpecs, tailSpecs,... 
            wing_index, horz_index, vert_index)
            obj.wing.coeff_liftA = wingSpecs(wing_index).CL_a;
            obj.wing.coeff_lift0 = wingSpecs(wing_index).C_L_0;
            obj.wing.coeff_lift_max = wingSpecs(wing_index).C_L_max;
            obj.wing.coeff_drag_data = table2array(wingSpecs(wing_index).C_D_data);
            obj.wing.thickness = wingSpecs(wing_index).th;
        
            obj.horz_stabiliser.coeff_liftA = tailSpecs(horz_index).CL_a;
            obj.horz_stabiliser.coeff_lift0 = tailSpecs(horz_index).C_L_0;
            obj.horz_stabiliser.coeff_lift_max = tailSpecs(horz_index).C_L_max;
            obj.horz_stabiliser.coeff_drag_data = table2array(tailSpecs(horz_index).C_D_data);
            obj.horz_stabiliser.thickness = wingSpecs(horz_index).th;
        
            obj.vert_stabiliser.coeff_liftA = tailSpecs(vert_index).CL_a;
            obj.vert_stabiliser.coeff_lift0 = tailSpecs(vert_index).C_L_0;
            obj.vert_stabiliser.coeff_lift_max = tailSpecs(vert_index).C_L_max;
            obj.vert_stabiliser.coeff_drag_data = table2array(tailSpecs(vert_index).C_D_data);
            obj.vert_stabiliser.thickness = wingSpecs(vert_index).th;
        end

        function link_constants(obj,constant_table)

            obj.static_margin_min_req = constant_table.static_margin_min_req;
            obj.static_margin_max_req = constant_table.static_margin_max_req;
            obj.num_power_vals = constant_table.num_power_vals;

            obj.units = constant_table.units;

            obj.physics = constant_table.physics;

            obj.course = constant_table.course;

            obj.score.baseline2 = constant_table.score.baseline2;
            obj.score.baseline3 = constant_table.score.baseline3;

            %obj.passengers.doll_length = constant_table.passengers.doll_length;
            %obj.passengers.doll_width = constant_table.passengers.doll_width;
            %obj.passengers.doll_mass = constant_table.passengers.doll_mass;

            obj.fuselage.density = constant_table.fuselage.density;
            obj.fuselage.wall_thickness = constant_table.fuselage.wall_thickness;
            obj.fuselage.tray_thickness = constant_table.fuselage.tray_thickness;
            obj.fuselage.object_spacing = constant_table.fuselage.object_spacing;
            obj.fuselage.interference_factor = constant_table.fuselage.interference_factor;
            
            obj.wing.area_density = constant_table.wing.density;
            obj.wing.sweep_angle25 = constant_table.wing.sweep_angle25;
            obj.wing.interference_factor = constant_table.wing.interference_factor;
            obj.wing.aspect_ratio_min = constant_table.wing.aspect_ratio_min;
            obj.wing.aspect_ratio_max = constant_table.wing.aspect_ratio_max;
            obj.wing.layer_thickness = constant_table.wing.layer_thickness;
            obj.horz_stabiliser.interference_factor = constant_table.horz_stabiliser.interference_factor;
            obj.horz_stabiliser.area_density = constant_table.horz_stabiliser.area_density;

            obj.vert_stabiliser.interference_factor = constant_table.vert_stabiliser.interference_factor;
            obj.vert_stabiliser.area_density = constant_table.vert_stabiliser.area_density;
            obj.vert_stabiliser.angle = constant_table.vert_stabiliser.angle;

            obj.misc_properties.load_factor = constant_table.misc_properties.load_factor;
            obj.misc_properties.time_safety = constant_table.misc_properties.time_safety;
            obj.misc_properties.propellor_efficiency = constant_table.misc_properties.propellor_efficiency;
            obj.misc_properties.electrical_efficiency = constant_table.misc_properties.electrical_efficiency;
            obj.misc_properties.propulsive_efficiency = constant_table.misc_properties.propulsive_efficiency;
            obj.misc_properties.density_PETG = constant_table.misc_properties.density_PETG;
            obj.misc_properties.stall_speed_max = constant_table.misc_properties.stall_speed_max;

            obj.battery.voltage = constant_table.battery.voltage;

            %obj.crew = constant_table.crew;

            %obj.msc.density = constant_table.msc.density;

            obj.nose.angle = constant_table.nose.angle;

            obj.servos = constant_table.servos;

            obj.guerney = constant_table.guerney;

            %obj.patient = constant_table.patient;
            
            obj.landing_gear = constant_table.landing_gear;

            obj.electronics = constant_table.electronics;
            
            %obj.trays.thickness = constant_table.trays.thickness;

        end


    end


end