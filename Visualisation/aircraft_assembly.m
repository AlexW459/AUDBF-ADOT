%Name: Alex
%Date: 2/8/24
%Description: Combines all existing parts so far into a single aircraft
%assembly

clear;
clc;
close all;

addpath('Empenage functions')
addpath('Fuselage functions')
addpath('Nosecone functions')
addpath('Tail functions')
addpath('Wing functions')
addpath('yFoil_classes')

tic

%Resolution of the plane, measured in points per metre
mesh_resolution = 30;

%Sets aircraft parameters
fuselage_height = 0.4;
fuselage_param_1 = 0.3;
fuselage_param_2 = 0.2;
fuselage_length = 1;
fuselage_inset = 0.1;


%length of nosecone
nosecone_length = 0.3;
%Height of largest part of solid section
nosecone_tip_scale = 0.5;
%Vertical distance between the centre of the fuselage and the4centre of the
%scaled down fuselage at the start of the solid section
nosecone_offset = -0.03;


%Length of wing
wing_length = 1.5;
%Vertical position of wing on the fuselage (fraction of fuselage height)
wing_vertical_position = 0.2;
%Horizontal position of wing along fuselage (fraction of fuselage length)
wing_horizontal_position = 0.6;
%Thickness of wing at the root and at the tip
wing_root_thickness = 0.1;
wing_scale  = 0.3;

wing_airfoil = "GOE567";


%Generates a new wing with the required resolution
generate_NACA_airfoil(wing_airfoil, 5.3, 4.95, 14.8, mesh_resolution, wing_root_thickness);


%Horizontal offset of the wing tip relative to the root
wing_sweep = 0;

%Length of tailcone
tailcone_length = 0.3;
%Height of curved tail section
tailcone_tip_scale = 0.3;

%Width of motor pods
motor_width = 0.08;

%Variables relating to empenage
horizontal_stabiliser_width = 0.5;
%Distance from the back of the plane ot the 
empennage_length = 0.2;
empennage_supports_thickness = 0.04;

horizontal_stabiliser_thickness = 0.06;

vertical_stabiliser_height = 0.4;
vertical_stabiliser_thickness = 0.06;

horizontal_stabiliser_airfoil = "NACA0012_horizontal_stabiliser";

vertical_stabiliser_airfoil = "NACA0012_vertical_stabiliser";

generate_NACA_airfoil(vertical_stabiliser_airfoil, 0, 0, 12, mesh_resolution, vertical_stabiliser_thickness);

generate_NACA_airfoil(horizontal_stabiliser_airfoil, 0, 0, 12, mesh_resolution, horizontal_stabiliser_thickness);


variable_list = {mesh_resolution, fuselage_height,...
    fuselage_param_1, fuselage_param_2, fuselage_length, fuselage_inset, nosecone_length, nosecone_tip_scale, ...
    nosecone_offset, wing_airfoil, wing_length, wing_vertical_position, wing_horizontal_position, ...
    wing_root_thickness, wing_scale, wing_sweep, tailcone_length, tailcone_tip_scale, ...
    motor_width, horizontal_stabiliser_width, empennage_length, empennage_supports_thickness, ...
    horizontal_stabiliser_thickness, horizontal_stabiliser_airfoil, vertical_stabiliser_height, ...
    vertical_stabiliser_thickness, vertical_stabiliser_airfoil};

variable_name_list = ["mesh_resolution", "fuselage_height", "fuselage_param_1", ...
    "fuselage_param_2", "fuselage_length", "fuselage_inset", "nosecone_length", ...
    "nosecone_tip_scale", "nosecone_offset",  "wing_airfoil", "wing_length",...
    "wing_vertical_position", "wing_horizontal_position", "wing_root_thickness", ...
    "wing_scale", "wing_sweep", "tailcone_length", "tailcone_tip_scale", ...
    "motor_width", "horizontal_stabiliser_width", "empennage_length", "empennage_supports_thickness", ...
    "horizontal_stabiliser_thickness", "horizontal_stabiliser_airfoil", "vertical_stabiliser_height", ...
    "vertical_stabiliser_thickness", "vertical_stabiliser_airfoil"];

profile_function_list = {@fuselage_profile, @wing_profile, @motor_pod_profile, ...
    @empennage_support_profile, @horizontal_stabiliser_profile, @vertical_stabiliser_profile};

profile_name_list = ["fuselage_profile", "wing_profile", "motor_pod_profile", ...
    "empennage_rod_profile", "horizontal_stabiliser_profile", "vertical_stabiliser_profile"];

aircraft = assembly(@extrude_fuselage, @fuselage_constraint, "fuselage", ...
    profile_function_list, profile_name_list, variable_list, variable_name_list, ...
    @calculate_derived_params);


aircraft.addPart(@extrude_nosecone, "nosecone", "fuselage", @nosecone_constraint, "base");

aircraft.addPart(@extrude_nosecone_cap, "nosecone_cap", "nosecone", @nosecone_cap_constraint, "base")

aircraft.addPart(@extrude_tailcone, "tailcone_base", "fuselage", @tailcone_constraint, "base");

aircraft.addPart(@extrude_wing_base_right, "wing_base_right", "fuselage", @wing_base_right_constraint, "base");

aircraft.addPart(@extrude_wing_base_left, "wing_base_left", "fuselage", @wing_base_left_constraint, "base");

aircraft.addPart(@extrude_motor_pod, "motor_pod_left", "wing_base_left",  @motor_pod_left_constraint, "base");

aircraft.addPart(@extrude_motor_pod, "motor_pod_right", "wing_base_right", @motor_pod_right_constraint, "base");

aircraft.addPart(@extrude_wing_tip_left, "wing_tip_left", "motor_pod_left", @wing_tip_left_constraint, "base");

aircraft.addPart(@extrude_wing_tip_right, "wing_tip_right", "motor_pod_right", @wing_tip_right_constraint, "base");

aircraft.addPart(@extrude_empennage_support, "empennage_support_right", "motor_pod_right", @empennage_support_constraint, "base");

aircraft.addPart(@extrude_empennage_support, "empennage_support_left", "motor_pod_left", @empennage_support_constraint, "base");


aircraft.addPart(@extrude_horizontal_stabiliser_right, "horizontal_stabiliser_right", "empennage_support_right", ...
    @horizontal_stabiliser_right_constraint, "base");

aircraft.addPart(@extrude_horizontal_stabiliser_left, "horizontal_stabiliser_left", "empennage_support_left", ...
    @horizontal_stabiliser_left_constraint, "base");

aircraft.addPart(@extrude_vertical_stabiliser, "vertical_stabiliser_right", "empennage_support_right",...
    @vertical_stabiliser_right_constraint, "base");

aircraft.addPart(@extrude_vertical_stabiliser, "vertical_stabiliser_;eft", "empennage_support_left",...
    @vertical_stabiliser_left_constraint, "base");



xlim([-2, 2])
ylim([-2, 2])
zlim([-2, 2])



%writeMeshtoObj(nodeCoordList, triangleNodeList, "Fuselage");

toc

aircraft.compute_params();
aircraft.compute_constraints();
aircraft.construct_profiles();
aircraft.construct_parts();
aircraft.transform_parts();
aircraft.show();

nodeCoordList = aircraft.parts{1}.nodes2coords;
triangleNodeList = aircraft.parts{2}.ext_facets2nodes;


nodeCoords = [];
% 
% for i = 1:size(aircraft.parts, 2)
%     nodeCoords = [nodeCoords; aircraft.transformed_parts{i}.nodes2coords(:, :)];
% end

