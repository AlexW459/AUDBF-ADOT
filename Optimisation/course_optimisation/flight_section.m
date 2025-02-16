%Name:    Isaac Nakone, Muhammad Rabay
%Date:    24/10/2024
%Purpose: This is a class definition for a flight section.
%         A flight section is a section of the aircraft trajectory
%         in which either thrust (T) or net acceleration (a) is held
%         constant.



classdef flight_section

    properties
        thrust       = [];     
        initial_time = [];
        final_time   = [];
        initial_condition = struct("velocity_x", [],"velocity_y", [], ...
            "velocity_z", [], "location_x", [], "location_y", [], ...
            "location_z", []);
        final_condition   = struct("velocity_x", [],"velocity_y", [], ...
            "velocity_z", [], "location_x", [], "location_y", [], ...
            "location_z", []);
        duration     = [];
        trajectory = struct("time", [], "velocity_x", [],"velocity_y", [], ...
            "velocity_z", [], "location_x", [], "location_y", [], ...
            "location_z", []);
        power  = [];   %Power = thrust*velocity;
        energy = [];   %integral(power)dt [consider efficiency]
        %NOTE: we will need to consider motor, propellor, battery
        %      efficiencies.
        aircraft_minimal = struct("payload_mass", [], "structural_mass",[], ...
                           "motor_battery_electronics_mass", [], ...
                           "lift_coefficient",[],"drag_coefficient",[], ...
                          "reference_area",[], ...
                          "motor_power_maximum", [], ...
                          "motor_efficiency", [],...
                          "battery_capacity", [], ...
                          "battery_efficiency", [],...
                          "payload_drag_coefficient", []); %Called aircraft_minimal to stress the fact that 
        %it's not the same as the aircraft model used in other aspects of
        %ADOT.
        has_payload    = [];  %Boolean == TRUE if still carrying payload, FALSE otherwise.
        is_taking_off  = [];  %Boolean == TRUE if section is taking off, FALSE otherwise.
        is_banking     = [];  %Boolean == TRUE if section is banking, FALSE otherwise.
        is_full_turn   = [];  %Boolean == TRUE if banking is 360 deg, FALSE if 180 deg or NOT banking at all.
        is_climbing    = [];  %Boolean == TRUE if aircraft is climbing, FALSE otherwise.
        speed_banking  = [];  % Only relevant if banking.
        radius_banking = [];  % Only relevant if banking.

        mass = [];             %Depends on whether payload is present or not.
        drag_coefficient = []; %Depends on whether taking off, and/or banking or not.
        lift_coefficient = []; %Depends on whether taking off or not  ( assume L == W in the case of banking).
        environment_physics = struct("air_density", 1.225, "gravity_acceleration",9.81,...
                                     "ground_drag_coefficient", []);
    end


    methods 

        function obj = flight_section(thrust, duration, initial_condition, initial_time, aircraft_minimal,...
                                      has_payload, is_taking_off, is_climbing, is_banking, is_full_turn,...
                                      radius_banking)
            
            obj.aircraft_minimal    = aircraft_minimal;
            obj.initial_condition   = initial_condition;
            obj.initial_time        = initial_time;
            obj.has_payload         = has_payload;
            obj.is_taking_off       = is_taking_off;
            obj.is_banking          = is_banking;
            obj.is_climbing         = is_climbing;
            obj.is_full_turn        = is_full_turn;
            if (obj.is_banking) 
                obj.speed_banking       = abs(obj.initial_condition.velocity_x); %Assume the banking speed is the initial speed.
            end
            obj.radius_banking      = radius_banking;
            obj.drag_coefficient    = obj.aircraft_minimal.drag_coefficient; %Initialise to the raw drag coefficient
            % - Because we will add the effects of take-off and payload.
            obj.lift_coefficient    = obj.aircraft_minimal.lift_coefficient;

            %If the aircraft has the payload, then the mass is bigger and
            %the coefficient of drag is (likely) to be bigger:
            if (obj.has_payload == true)
                obj.mass = obj.aircraft_minimal.payload_mass + ...
                           obj.aircraft_minimal.structural_mass + ...
                           obj.aircraft_minimal.motor_battery_electronics_mass;

                obj.drag_coefficient = obj.drag_coefficient + ...
                                       obj.aircraft_minimal.payload_drag_coefficient;
            else
                obj.mass = obj.aircraft_minimal.structural_mass + ...
                           obj.aircraft_minimal.motor_battery_electronics_mass;

            end

            if (obj.is_taking_off)
                obj.environment_physics.ground_drag_coefficient = 1.5; %INSERT formula here for ground drag (can be an estimate). [TODO]
                obj.drag_coefficient =  obj.drag_coefficient+obj.environment_physics.ground_drag_coefficient;
                obj.drag_coefficient =  obj.drag_coefficient+obj.environment_physics.ground_drag_coefficient;
            end


            %If the aircraft is banking, then the thrust is equal to the
            % drag, which is given by T == D == 0.5*C_D_turn*S_ref*air_density*V_turn^2.
            if (obj.is_banking)
                obj.speed_banking = speed_banking;
                turning_drag_correction = 0.1; %INSERT formula here for turning drag correction. [TODO]
                obj.drag_coefficient = obj.drag_coefficient + ...
                                       turning_drag_correction;

                obj.thrust = 0.5*obj.drag_coefficient*obj.environment_physics.air_density*...
                             obj.aircraft_minimal.reference_area*obj.speed_banking^2;
            else
                obj.thrust = thrust; %If it is not banking then we just take the input thrust.
            end


            if ( (~obj.is_banking) && (~obj.is_taking_off) ) %This implies the aircraft is in level flight.
                obj.duration = duration;
            end

            %NOW: call the function associated with determining the
            % trajectory based on the initial conditions and the thrust:
            obj = calculate_trajectory(obj);


        end


        function obj = calculate_trajectory(obj)

            options = optimset('Display','off');
            

            if (obj.is_banking)

                if (obj.is_full_turn)
                    turn_arc = 2*pi*obj.radius_banking;
                else
                    turn_arc = pi*obj.radius_banking;
                end

                obj.duration = turn_arc/obj.speed_banking; %Assuming constant speed turn.

                obj.final_time = obj.initial_time + obj.duration;

                obj.trajectory.time = linspace(obj.initial_time, obj.final_time, 100); %Replace the magic number (100) here by a variable. [TODO].
                

                %Take the direction vector of the initial condition and
                %rotate by 90 deg counterclockwise( looking from a
                %bird's eye view):

                initial_direction = [obj.initial_condition.velocity_x,obj.initial_condition.velocity_y, 0 ]; %neglect any climb rate.
                initial_direction = initial_direction/norm(initial_direction,2); %Make it a unit vector.


                    
                if (obj.is_full_turn)

                    turn_direction = ([0, -1  0;...
                                        1, 0  0;...
                                        0, 0, 1]*(initial_direction'))'; %Assume full turns are always right turns.
                    %Here this is a 90 degree clockwise rotation
                    %matrix.

                    %Determine the final location
                    obj.final_condition.location_x = obj.initial_condition.location_x; %Assuming the x-direction is aligned with the straight sections.
                    obj.final_condition.location_y = obj.initial_condition.location_y;
                    obj.final_condition.location_z = obj.initial_condition.location_y;

                    %Determine the final velocity:
                    obj.final_condition.velocity_x = obj.initial_condition.velocity_x; %For a full turn the velocity remains unchanged.
                    obj.final_condition.velocity_y = obj.initial_condition.velocity_y;
                    obj.final_condition.velocity_y = obj.initial_condition.velocity_z;


                else

                    turn_direction = ([0, 1  0;...
                                       -1, 0  0;...
                                        0, 0, 1]*(initial_direction'))'; %Assume half turns are always left turns.
                    %Here this is a 90 degree anti-clockwise rotation
                    %matrix.

                    %Determine the final location:
                    obj.final_condition.location_x = obj.initial_condition.location_x; %Assuming the x-direction is aligned with the straight sections.
                    obj.final_condition.location_y = obj.initial_condition.location_y +...
                                                     2.0*obj.radius_banking*turn_direction(2);
                    obj.final_condition.location_z = obj.initial_condition.location_y;

                    %Determine the final velocity:
                    obj.final_condition.velocity_x = -obj.initial_condition.velocity_x; %For a half turn the velocity x-component is made negative.
                    obj.final_condition.velocity_y = obj.initial_condition.velocity_y;
                    obj.final_condition.velocity_y = obj.initial_condition.velocity_z;


                end  


                %Determine the trajectory based on the banking
                %orientation (left or right):

                %Locate the center of the turn circle:
                turn_circle_center = [obj.final_condition.location_x, obj.final_condition.location_y, obj.final_condition.location_z]+...
                                     1.0*obj.radius_banking*turn_direction;

                if (obj.is_full_turn) 

                    angle = linspace(0.0, 2.0*pi, size(obj.trajectory.time,1) );

                    obj.trajectory.location_x = turn_circle_center(1)-obj.radius_banking*sin(angle);
                    obj.trajectory.location_y = turn_circle_center(2)-obj.radius_banking*cos(angle);
                    obj.trajectory.location_z = obj.initial_condition.location_z;

                    obj.trajectory.velocity_x = -obj.speed_banking*cos(angle);
                    obj.trajectory.velcoity_y =  obj.speed_banking*sin(angle);
                    obj.trajectory.velocity_z = zeros(1,size(obj.trajectory.time,1));


                else

                    angle = linspace(0.0, pi, size(obj.trajectory.time,1) );

                    obj.trajectory.location_x = turn_circle_center(1)-obj.radius_banking*cos(angle);
                    obj.trajectory.location_y = turn_circle_center(2)-obj.radius_banking*sin(angle);
                    obj.trajectory.location_z = obj.initial_condition.location_z;

                    obj.trajectory.velocity_x = sign(initial_direction(1))*obj.speed_banking*cos(angle);
                    obj.trajectory.velcoity_y = sign(initial_direction(1))*obj.speed_banking*sin(angle);
                    obj.trajectory.velocity_z = zeros(1,size(obj.trajectory.time,1));

                    %NOTE: This will work for both the left and right turns in
                    %      the course.


                end
            else
                if (obj.is_taking_off) %The take off period ends when the lift matches the weight, L == W == m*g.

                    %Compute the final velocity x based on the condition
                    %that L == 1.1*W == m*g;
                    obj.final_condition.velocity_x = sqrt(2*obj.mass*obj.environment_physics.gravity_acceleration/...
                                                          (obj.lift_coefficient*obj.environment_physics.air_density*...
                                                           obj.aircraft_minimal.reference_area));
                    obj.final_condition.velocity_y = 0.0;
                    obj.final_condition.velocity_z = 0.0;

                    %Recall that the acceleration is equal to a ==
                    %(T-D)/mass: But D == D(v) == 0.5*C_D*rho*S_ref*v^2 is
                    %a function of velocity.

                    is_moving  = @(v)  (obj.thrust > 0.5*obj.drag_coefficient*obj.environment_physics.air_density*...
                                                          obj.aircraft_minimal.reference_area*v.^2);

                    acceleration_x = @(v) is_moving(v).*(obj.thrust - 0.5*obj.drag_coefficient*obj.environment_physics.air_density*...
                                                          obj.aircraft_minimal.reference_area*v.^2)/obj.mass;

                    rax = @(v) 1./acceleration_x(v); %define the reciprocal function.

                    obj.duration = integral(rax, 0, obj.final_condition.velocity_x); %integral (1/a(v)) dv == Delta t.

                    obj.final_time = obj.initial_time + obj.duration;

                    obj.trajectory.time = linspace(obj.initial_time,obj.final_time, 100); %The sequence of time steps.
                    
                    dt = obj.trajectory.time(2) - obj.trajectory.time(1); %The time jump

                    obj.trajectory.velocity_x = zeros(1,size(obj.trajectory.time,2));

                    for i = 1:(size(obj.trajectory.time,2)-1)

                        obj.trajectory.velocity_x(i+1) = fsolve(@(v)(integral(rax,obj.trajectory.velocity_x(i),v)-dt),...
                                                                obj.trajectory.velocity_x(i),options);
                    end

                    obj.trajectory.location_x = cumtrapz(obj.trajectory.time, obj.trajectory.velocity_x); %integrate the velocity to get position.
                    obj.trajectory.location_y = zeros(1,size(obj.trajectory.time,2));
                    obj.trajectory.location_z = zeros(1,size(obj.trajectory.time,2));

                    


                    obj.trajectory.velocity_y = zeros(1,size(obj.trajectory.time,2));
                    obj.trajectory.velocity_z = zeros(1,size(obj.trajectory.time,2));

                else %This case is straight section at constant thrust:

                    

                    %Recall that the acceleration is equal to a ==
                    %(T-D)/mass: But D == D(v) == 0.5*C_D*rho*S_ref*v^2 is
                    %a function of velocity.

                    

                    acceleration_x = @(v) sign(obj.initial_condition.velocity_x)*...
                                          (obj.thrust - 0.5*obj.drag_coefficient*obj.environment_physics.air_density*...
                                           obj.aircraft_minimal.reference_area*v.^2)/obj.mass;

                    rax = @(v) 1./acceleration_x(v); %define the reciprocal function.

                    obj.final_time = obj.initial_time + obj.duration;

                    obj.trajectory.time = linspace(obj.initial_time,obj.final_time, 100); %The sequence of time steps.
                    
                    dt = obj.trajectory.time(2) - obj.trajectory.time(1); %The time jump

                    obj.trajectory.velocity_x = zeros(1,size(obj.trajectory.time,2));

                    obj.trajectory.velocity_x(1) = obj.initial_condition.velocity_x;

                    

                    for i = 1:(size(obj.trajectory.time,2)-1)

                        obj.trajectory.velocity_x(i+1) = fsolve(@(v)(integral(rax,obj.trajectory.velocity_x(i),v)-dt),...
                                                                obj.trajectory.velocity_x(i),options);
                    end

                    obj.trajectory.location_x = cumtrapz(obj.trajectory.time, obj.trajectory.velocity_x); %integrate the velocity to get position.
                    obj.trajectory.location_y = zeros(1,size(obj.trajectory.time,2));
                    


                    obj.trajectory.velocity_y = zeros(1,size(obj.trajectory.time,2));
                    

                    if (obj.is_climbing)
                        %Recall the acceleration in the z-direction is az =
                        %(L-W)/mass.

                        acceleration_z = @(v) (0.5*obj.lift_coefficient*obj.environment_physics.air_density*...
                                                          obj.aircraft_minimal.reference_area*v.^2-...
                                                          obj.mass*obj.environment_physics.gravity_acceleration)/obj.mass;

                        raz = @(v) 1./acceleration_z(v); %define the reciprocal function.

                        obj.trajectory.velocity_z = zeros(1,size(obj.trajectory.time,2));

                        %Note initial velcoity in the z-direction is 0.

                        

                        for i = 1:(size(obj.trajectory.time,2)-1)

                            obj.trajectory.velocity_z(i+1) = fsolve(@(v)(integral(raz,obj.trajectory.velocity_z(i),v)-dt),...
                                                                obj.trajectory.velocity_z(i),options);
                        end

                        obj.trajectory.location_z = cumtrapz(obj.trajectory.time, obj.trajectory.velocity_z);

                    else %In the instance where we are not climbing.

                        obj.trajectory.velocity_z = zeros(1,size(obj.trajectory.time,2));
                        obj.trajectory.location_z = obj.initial_condition.location_z*ones(1,size(obj.trajectory.time,2));

                    end




                end

            end

            %For all flight types, the computation of power and energy will
            %be identical:

            obj.power = obj.thrust*sqrt(obj.trajectory.velocity_x.^2 +...
                                    obj.trajectory.velocity_y.^2 +...
                                    obj.trajectory.velocity_z.^2); %Note thrust is constant but velocity is not.
            obj.energy = trapz(obj.trajectory.time, obj.power); %integrate the power over time to get the energy consumed.

            obj.final_condition.location_x = obj.trajectory.location_x(end);
            obj.final_condition.location_y = obj.trajectory.location_y(end);
            obj.final_condition.location_z = obj.trajectory.location_z(end);

            obj.final_condition.velocity_x = obj.trajectory.velocity_x(end);
            obj.final_condition.velocity_y = obj.trajectory.velocity_y(end);
            obj.final_condition.velocity_z = obj.trajectory.velocity_z(end);

        end

        
       

    end



end