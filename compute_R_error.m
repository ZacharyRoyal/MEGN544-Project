function error = compute_R_error(T, T_soln)
    R = get_R(T);
    R_soln = get_R(T_soln);

    error = rot2AngleAxis(R*R_soln');
end