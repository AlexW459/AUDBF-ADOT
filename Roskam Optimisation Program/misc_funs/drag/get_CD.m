function C_D = get_CD(C_D_data, C_L_0, CL_a, C_L_cr)

    
    alpha = ceil((C_L_cr-C_L_0)/CL_a);

   
    if (alpha <=-30)
        C_D = C_D_data(1,1);

    elseif (alpha >=-30 && alpha < 31)

        C_D = C_D_data(alpha+30,1);
    else
        C_D = C_D_data(end,1);
    end
end