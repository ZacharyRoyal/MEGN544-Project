function error = compute_pose_error(T_desired, T_soln)
    error = [compute_disp_error(T_desired, T_soln);
        compute_R_error(T_desired, T_soln)];
end