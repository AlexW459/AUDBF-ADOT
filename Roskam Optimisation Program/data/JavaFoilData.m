function aerofoil_data = JavaFoilData(type)
    min_angle = 8;                                                              %Set minimum angle ofr C_L_max
    if(type == 'wing')
        %If the wing aerofils are required, get data from all wing aerofoils
        excelFileName(1) = "./data/EPPLER 398";
        excelFileName(2) = "./data/GOE 567";
        excelFileName(3) = "./data/NACA 2112";
        excelFileName(4) = "./data/NACA 6412";
        excelFileName(5) = "./data/Clark Y";
    else
        %If tail aerofils are required, get data from all tail aerofoils
        excelFileName(1) = "./data/NACA 0009";
        excelFileName(2) = "./data/NACA 0012";
    end

     
    
    num_aerofoils = 1;                                                          %Initialise current number of aerofils processed
    for i=1:length(excelFileName)
        %finding CL/alpha slope
        initTable = readtable(excelFileName(i), 'Sheet', 1);                    %Look at data at 0 flap size
        t_length = size(initTable.Cl);                                          %Find number of C_L values in database
        t_length = t_length(1);
        C_L_0 = initTable.Cl(32);                                               %Find initial C_L at 0 angle of attack and 0 flap deflection
        CL_max = max(initTable.Cl);                                             %Find max of table with 0 flap size to find C_L_alpha slope
        slope = (CL_max - C_L_0)/(t_length/2);                                  %Find C_L_alpha slope
        %finding CD/alpha data
        C_D_data = initTable.Cd;                                                %Read in the data for the coefficients of drag
        alpha = initTable.x_;
        %find max for 4 different flap sizes
        [~, sheetNames] = xlsfinfo(excelFileName(i));
        for j=2:4
            currentTable = readtable(excelFileName(i), 'Sheet', j);             %Read in Each Flap deflection angle data for a given flap size
            aerofoil.th = initTable.Max_Thickness(1);
            aerofoil.CL_a = slope;                                              %Set aerfoil C_L_alpha
            aerofoil.C_L_0 = C_L_0;                                             %Set C_L value at 0 angle of attack
            aerofoil.C_L_max = max(max(max(currentTable.Cl(min_angle + 30 : 60),... %Find C_L_max
                currentTable.Cl_1(min_angle + 30 : 60)), ...
                currentTable.Cl_2(min_angle + 30 : 60)));
            aerofoil.C_D_data = table(C_D_data, alpha);                                       %Add C_D data to aerfoil struct to be able to calculate the C_D
            aerofoil.C_D_data(1,:) = [];
            aerofoil.flap_size = sheetNames(j);
            aerofoil_data(num_aerofoils) = aerofoil;                            %Add aerofoil to aerofoil table
            num_aerofoils = num_aerofoils+1;                                    %Incremenet number of aerfoils in table
        end 
    end



end