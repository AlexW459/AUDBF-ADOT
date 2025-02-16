%Name:        Isaac Nakone, Denis Vasilyev, Harry Rowton, Jingya Liu
%Date:        15/12/2023
%Description: Master code for the ADOT calls all the sub-functions.
%generates and optimises designs.
%Usage: The following should be adjusted based on requirenments before
%using the code: 
%   Battery, motor and aerofoil database indexes, removing any
%   unreasonably powerful or weak components will speed up the code as it will
%   egenrate less impossible designs.
%
%   N_valid, this is the number of seeds to optimise, the more seeds are
%   used the more likely it is that the final design is close to the
%   theoretically best possible design. Using at least 1000 will give a
%   pretty good design in most cases but idealy 10000+ should be used if
%   possible
%






%%Initialisation
clear;
clc;
close;
rng("shuffle");

%Set this ranges var up as a table!!!
ranges = [  0.7048	1;      % obj.wing.taper_ratio             = opt_vars(1);
            0.1	0.4;        % obj.wing.chord_root              = opt_vars(2);
            0.1	0.5;        % obj.fuselage.width               = opt_vars(3);
            0.8	1.5;        % obj.wing.span                    = opt_vars(4);
            0.2	0.6;        % obj.wing.position                = opt_vars(5);
            0.12	0.2;    % obj.horz_stabiliser.chord_root   = opt_vars(6);
            0.762	0.762;  % obj.horz_stabiliser.span         = opt_vars(7);
            0.75	1.5;    % obj.misc_properties.aspect_ratio = opt_vars(8);
            10	30;         %cruise speed 2                    = opt_vars(9);
            10	30;         %cruise speed 3                    = opt_vars(10);
            0.1	1;          %cruise acceleration 2             = opt_vars(11);
            0.1	1;          %cruise acceleration 2             = opt_vars(12);
            0.1 0.2;        %obj.x1_prototype.mass             = opt_vars(13);
            0.1 5;]         %obj.fuel_tank.mass                = opt_vars(14);


%Add subdirectories:
addpath('./misc_funs');
addpath('./data');
addpath('./con_funs');
addpath('./classes');

%Add sub-subdirectories:
addpath('./misc_funs/scoring');
addpath('./misc_funs/mass_funs');
addpath('./misc_funs/dimensions');
addpath('./misc_funs/cruise');
addpath('./misc_funs/center_of_gravity');
addpath('./misc_funs/drag');

%Suppresses warning from readtable:
warning('OFF', 'MATLAB:table:ModifiedAndSavedVarnames'); 

%Setting up constant data from databases
constants = constant_table;

%load in the battery table:
batterySpecs = readtable("BatterySpecs.xlsx");

%load in the motor table:
motorSpecs = readtable("MotorSpecs.xlsx");

%load in the airfoil tables:
wingSpecs = JavaFoilData('wing');
tailSpecs = JavaFoilData('tail');

%Set the constraints as defined in the constraint_set.m class
constraints = constraint_set;


%
%SET N_valid to be the number of seeds to optimise. More seeds will result
%in a better design. I recommend using at least 1000 seeds, but more is
%better
%
N_valid = 10;


%Initialise constants that track the number of seeds completed (count), as well as
%number of seeds that attempted optimisation (N)
count = 0;
N = 0;

%Set the options for the fmincon function
options1=optimoptions(@fmincon,'Display','off', 'ConstraintTolerance', 0);


%%Running the optimisation:
tic
while (count < N_valid)
    % disp(N); %uncomment this if you want to see how many seeds it has to
    % try before it is able to find oen that can be optimised
    N = N + 1;

    
    %create a new empty aircraft object 
    plane = aircraft;
 
    %Choose the battery, motor, wing, and tail randomly:
    battery_index = randi([4 26],1);
    motor_index   = randi([6 6],1); %This is based on the scavenged motor.
    wing_index    = randi([1 15],1);
    horz_index    = randi([1 6],1);
    vert_index    = horz_index;

    %Link the constants to the aircraft:
    plane.link_constants(constants)

    %Link the battery to the aircraft:
    plane.link_battery( battery_index, batterySpecs);

    %Link the motor to the aircraft:
    plane.link_motor( motor_index, motorSpecs);

    %Link the airfoils to the aircraft:
    plane.link_airfoils(wingSpecs, tailSpecs,... 
                        wing_index, horz_index, vert_index);

    
    %generate some random params on interval [0,1]:
    %The random number is used to decide what the starting point for hte
    %optimisiation parameters will be
    opt_vars0 = (ranges(:,2)-ranges(:,1)).*rand(size(ranges,1),1) + ranges(:,1);

    %TODO remove stabvars and properly calculate the msc center of
    %gravity. I dont know why it was being set ot always be 0.55 m but htis
    %is likely a relic of the stabilisation code attempt
    stab_vars = 0.55;
    
    
    %Perform optimisation using fmincon. Output is the optimised variables
    %for that aircraft, the score, and the exitflag which defines if it was
    %able to meet all constraints.
    [opt_vars1,S, exitflag] = fmincon(@(opt_vars)(-1)*score(plane, opt_vars,stab_vars ),opt_vars0,[],[],[],[],ranges(:,1),ranges(:,2),...
                @(opt_vars)constraints.eval_constraints(opt_vars,stab_vars, plane),options1);
    
    %Link the optimised variables to the aircraft
    plane.link_opt_vars(opt_vars1);
    
    %Link the score to the aircraft
    plane.score.total = S;
    
    %exitflag is 0 if it failed to meet one or more constraints, 1 if it
    %was able to meet them all
    if(exitflag == 1)
        toc
        tic
        %Increment number of valid seeds processed
        count = count + 1;
        %Save the optimised design in an array of aircraft
        valid(count) = plane; 
        %Display the number of seeds processed to help keep track of
        %progress
        disp(count);
    end

 
   
end

%save the optimised aircraft in a file
filename = "optimised_designs.mat";
save(filename,"valid")

disp("done");



%use this to look at specific   values in all designs for debugging
% for i = 1:10
%     disp(valid(i).score.total);
% end

