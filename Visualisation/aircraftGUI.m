%Name: Alex
%Date: 2/8/24
%Description: Combines all existing parts so far into a single aircraft
%assembly and allows the user to modify aspects of the plane using a GUI


clear;
clc;
close all force;

addpath('distmesh-master')
addpath('Empenage functions')
addpath('Fuselage functions')
addpath('Nosecone functions')
addpath('Tail functions')
addpath('Wing functions')
addpath('yFoil_classes')


%Sets the list of materials and their densities
material_table_file = "Materials.csv";
material_table = readtable(material_table_file);
material_table.Properties.VariableUnits = {};

PETG = 2;
GAROLITE = 1;
ASA = 3;


%Sets the list of motors to choose from
motor_table_file = "Motor_Selection.csv";
motor_table = readtable(motor_table_file);
motor_table_units = {'', 'mm', 'mm', 'V', 'A', 'g', 'g'};
motor_table.Properties.VariableUnits = motor_table_units;

%Sets the list of batteries to choose from
battery_table_file = "Battery_Selection.csv";
battery_table = readtable(battery_table_file);
battery_table_units = {'', 'V', 'g', 'mAh', '1/h', 'AUD'};
battery_table.Properties.VariableUnits = battery_table_units;


%Sets the list of airfoils to choose from
airfoil_table_file = "Airfoils.csv";
airfoil_table = readtable(airfoil_table_file);
airfoil_units = {'', 'percentile', 'decile', 'percentile'};
airfoil_table.Properties.VariableUnits = airfoil_units;



aircraft_parameters = readtable("Initial_values.csv");

variable_list = aircraft_parameters{1, :};

variable_name_list = [""];

for i = 1:size(aircraft_parameters.Properties.VariableNames, 2)
    variable_name_list(1, i) = convertCharsToStrings(aircraft_parameters.Properties.VariableNames{i});
end





profile_function_list = {@fuselage_profile, @wing_profile, @motor_pod_profile, ...
    @empennage_support_profile, @horizontal_stabiliser_profile, @vertical_stabiliser_profile};

profile_name_list = ["fuselage_profile", "wing_profile", "motor_pod_profile", ...
    "empennage_support_profile", "horizontal_stabiliser_profile", "vertical_stabiliser_profile"];

aircraft = assembly(@extrude_fuselage, @fuselage_constraint, "fuselage", ...
    ASA, profile_function_list, profile_name_list, variable_list, variable_name_list, ...
    @calculate_derived_params, material_table, motor_table, battery_table, airfoil_table);

aircraft.addPart(@extrude_nosecone, "nosecone", "fuselage", @nosecone_constraint, ASA);

aircraft.addPart(@extrude_tailcone, "tailcone", "fuselage", @tailcone_constraint, ASA);

aircraft.addPart(@extrude_wing_base_right, "wing_base_right", "fuselage", @wing_base_right_constraint, ASA);

aircraft.addPart(@extrude_wing_base_left, "wing_base_left", "fuselage", @wing_base_left_constraint, ASA);

aircraft.addPart(@extrude_motor_pod, "motor_pod_left", "wing_base_left",  @motor_pod_left_constraint, ASA);

aircraft.addPart(@extrude_motor_pod, "motor_pod_right", "wing_base_right", @motor_pod_right_constraint, ASA);

aircraft.addPart(@extrude_wing_tip_left, "wing_tip_left", "motor_pod_left", @wing_tip_left_constraint, ASA);

aircraft.addPart(@extrude_wing_tip_right, "wing_tip_right", "motor_pod_right", @wing_tip_right_constraint, ASA);

aircraft.addPart(@extrude_empennage_support, "empennage_boom_right", "motor_pod_right", @empennage_support_constraint, ASA);

aircraft.addPart(@extrude_empennage_support, "empennage_boom_left", "motor_pod_left", @empennage_support_constraint, ASA);

aircraft.addPart(@extrude_empennage_rod, "empennage_rod_left", "motor_pod_left", @empennage_rod_constraint, ASA);

aircraft.addPart(@extrude_empennage_rod, "empennage_rod_right", "motor_pod_right", @empennage_rod_constraint, ASA);


aircraft.addPart(@extrude_horizontal_stabiliser, "horizontal_stabiliser_right", "empennage_rod_right", ...
    @horizontal_stabiliser_right_constraint, PETG);

aircraft.addPart(@extrude_horizontal_stabiliser, "horizontal_stabiliser_left", "empennage_rod_left", ...
    @horizontal_stabiliser_left_constraint, PETG);

aircraft.addPart(@extrude_vertical_stabiliser, "vertical_stabiliser_right", "empennage_rod_right",...
    @vertical_stabiliser_right_constraint, PETG);

aircraft.addPart(@extrude_vertical_stabiliser, "vertical_stabiliser_left", "empennage_rod_left",...
    @vertical_stabiliser_left_constraint, PETG);



function add_ui_to_window(aircraft_object)

    aircraft_object.compute_params();
    aircraft_object.compute_constraints();
    aircraft_object.construct_profiles();
    aircraft_object.construct_parts();
    aircraft_object.transform_parts();
    aircraft_object.generate_surface();


    window = uifigure('Name','Airplane model', 'NumberTitle','off', 'Position', [300, 100, 800, 500]);


    figurepanel = uipanel(window, 'Position',[0, 0, window.Position(3)*5/8, window.Position(4)]);
    ax = axes(figurepanel);

    view(ax, 135, 35);
    
    xlim(ax, [-1,1]);
    ylim(ax, [-1,1]);
    zlim(ax, [-1,1]);

    aircraft_object.show(ax);


    slider_panel=uipanel(window, 'Position',...
        [window.Position(3)*5/8, window.Position(4)*1/6, ...
        window.Position(3)*3/8, window.Position(4)*5/6], 'Scrollable','on');

    min_max_values = [5, 50; %mesh resolution
                      0.1, 0.5; %fuselage height
                      0.1, 0.5; %fuselage param 1
                      0.1, 0.5; %fuselage param 2
                      0.5, 2; %fuselage length
                      0.005, 0.3; %plastic thickness
                      0.2, 0.6; %nosecone length
                      0.2, 0.9; %nosecone tip scale
                      -0.15, 0.15; %nosecone offset
                      0, 0; %wing airfoil index
                      0.4, 2; %wing length
                      0, 1; %wing vertical position
                      0, 1; %wing horizontal position
                      0.03, 0.2; %wing root thickness
                      0.2, 1; %wing scale
                      0, 1; %wing_sweep
                      0.2, 0.5; %tailcone length
                      0.1, 0.9; %tailcone tip scale
                      0, 0; %battery_index
                      0, 1; %battery_pos
                      0, 0; %motor index
                      0.1, 0.8; %horizontal stabiliser width
                      0.05, 0.6; %empennage length
                      0.02, 0.06; %empennage supports thickness
                      0.02, 0.08; %horizontal stabiliser thickness
                      0, 0; %horizontal stabiliser airfoil index
                      0.1, 0.6; %vertical stabiliser height
                      0.02, 0.08; %vertical stabiliser thickness
                      0, 0 %vertical stabiliser airfoil index
                      ]; 

    default_values = aircraft_object.aircraft_parameters;

    numSliders = size(aircraft_object.aircraft_parameters, 2);

    sliderPos = 30;
    sliderHeight = 45;
    textPos = sliderPos - 20;

    sliders = cell(numSliders);
    editable_boxes = cell(numSliders);

    for slider_num = 1:numSliders
        if(aircraft_object.parameter_names(slider_num) ~= "horizontal_stabiliser_airfoil_index" && ...
            aircraft_object.parameter_names(slider_num) ~= "vertical_stabiliser_airfoil_index" && ...
            aircraft_object.parameter_names(slider_num) ~= "wing_airfoil_index" && ...
            aircraft_object.parameter_names(slider_num) ~= "motor_index" && ...
            aircraft_object.parameter_names(slider_num) ~= "battery_index")

            sliders{slider_num} = uicontrol(slider_panel, 'style','slider','Position',[30, sliderPos, 200, 20],...
            'min',min_max_values(slider_num, 1),'max', min_max_values(slider_num, 2), ...
            'Value', default_values(slider_num), 'Callback', @slider_changed);

            textH = uicontrol(slider_panel, 'style','text', 'position',[10, textPos, 200, 20]);
            textH.String = strcat(aircraft_object.parameter_names(slider_num), ": ");

            textH = uicontrol(slider_panel, 'style','text', 'position',[0, sliderPos, 30, 20]);
            textH.String = min_max_values(slider_num, 1);
            textH = uicontrol(slider_panel, 'style','text', 'position',[230, sliderPos, 30, 20]);
            textH.String = min_max_values(slider_num, 2);
            
            editable_boxes{slider_num} = uieditfield(slider_panel, 'Position', [190, textPos, 70, 20],...
                'Value', num2str(default_values(slider_num)), 'ValueChangedFcn', @text_changed);
            
            
            sliderPos = sliderPos + sliderHeight;
            textPos = sliderPos - 20;
        end
    end

    uicontrol(slider_panel, 'style','text', 'position',[0, textPos, 200, 10], 'String', "");
    
    button_panel = uipanel(window, 'Position',...
        [window.Position(3)*5/8, 0, window.Position(3)*3/8, ...
        window.Position(4)*1/6]);

    button = uibutton(button_panel, 'Position', [100, 25, 80, 50], ...
        'ButtonPushedFcn', @(source,eventdata)render, 'Text', "Render");
        
    function slider_changed(~ ,~)
        for text_num = 1:numSliders
            if(aircraft_object.parameter_names(text_num) ~= "horizontal_stabiliser_airfoil_index" && ...
                aircraft_object.parameter_names(text_num) ~= "vertical_stabiliser_airfoil_index" && ...
                aircraft_object.parameter_names(text_num) ~= "wing_airfoil_index" && ...
                aircraft_object.parameter_names(text_num) ~= "motor_index" && ...
                aircraft_object.parameter_names(text_num) ~= "battery_index")
                
                editable_boxes{text_num}.Value = num2str(sliders{text_num}.Value);
            end
        end

    end

    function text_changed(~, ~)
        for text_num = 1:numSliders
            if(aircraft_object.parameter_names(text_num) ~= "horizontal_stabiliser_airfoil" && ...
                aircraft_object.parameter_names(text_num) ~= "vertical_stabiliser_airfoil" && ...
                aircraft_object.parameter_names(text_num) ~= "wing_airfoil" && ...
                aircraft_object.parameter_names(text_num) ~= "motor_index" && ...
                aircraft_object.parameter_names(text_num) ~= "battery_index")

                editable_boxes{text_num}.Value = strtrim(editable_boxes{text_num}.Value);

                newValue = str2double(editable_boxes{text_num}.Value);

                if newValue < sliders{text_num}.Min
                    sliders{text_num}.Value = sliders{text_num}.Min;
                    editable_boxes{text_num}.Value = num2str(sliders{text_num}.Min);
                elseif newValue > sliders{text_num}.Max
                    sliders{text_num}.Value = sliders{text_num}.Max;
                    editable_boxes{text_num}.Value = num2str(sliders{text_num}.Max);
                else
                    sliders{text_num}.Value = newValue;
                end
            end
        end
    end

    function render(~, ~)

        cla(ax);

        for param_num = 1:numSliders
            if(aircraft_object.parameter_names(param_num) ~= "horizontal_stabiliser_airfoil_index" && ...
                aircraft_object.parameter_names(param_num) ~= "vertical_stabiliser_airfoil_index" && ...
                aircraft_object.parameter_names(param_num) ~= "wing_airfoil_index" && ...
                aircraft_object.parameter_names(param_num) ~= "motor_index" && ...
                aircraft_object.parameter_names(param_num) ~= "battery_index")

                aircraft_object.aircraft_parameters(param_num) = sliders{param_num}.Value;

            end
        end

    
        aircraft_object.compute_params();
        aircraft_object.compute_constraints();
        aircraft_object.construct_profiles();
        aircraft_object.construct_parts();
        aircraft_object.transform_parts();

        aircraft_object.show(ax);

    end

end

add_ui_to_window(aircraft);