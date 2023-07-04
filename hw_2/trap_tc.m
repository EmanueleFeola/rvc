function [t,q,q_dot,q_ddot] = trap_tc(Ts, qi, qf, ti, tf, tc)
    qc_ddot = (qf - qi) / (tc*(tf-ti) - tc^2);
    qc_dot = qc_ddot * tc;
    
    %% check
    % feasibility check
    if check_feasible(ti, tf, tc) == 1
        fprintf("traj is not feasible: tc > (tf-ti)/2\n");
        return;
    end
    
    % triangular shape check
    if check_triangular(qi, qf, qc_dot, qc_ddot) == 1
        fprintf("traj is triangular\n");
    end

    %% call generic trap traj planner with initial velocities set to zero, and ta = td
    qi_dot = 0;
    qf_dot = 0;
    ta = tc;
    td = tc;
    [t,q,q_dot,q_ddot] = trap_not_null(Ts, qi, qf, ti, tf, ta, td, qc_dot, qi_dot, qf_dot);
end