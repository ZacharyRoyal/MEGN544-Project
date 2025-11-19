function error = compute_disp_error(T, T_soln)
    p_T = get_displacement(T);
    P_T_soln = get_displacement(T_soln);

    error = p_T - P_T_soln;
end