function [] = trap_multipoint_2(points, Ts, qc_ddot_max, qc_dot_max)
new_points = [points(1, 1), points(1, 2)];
tot_time = [];
tot_q = [];
tot_q_dot = [];
tot_q_ddot = [];

qf_prec = 0;
tf_prec = points(1, 2);
for i = 1:size(points, 1)-1
    qi = points(i, 1);
    qf = points(i+1, 1);

    % non-null intermediate velocities
    qi_dot = qf_prec;
    qf_dot = 0;
    if i < size(points, 1) - 1
        sign_delta_q_k = sign(qf - qi);
        sign_delta_q_k1 = sign(points(i+2, 1) - qf);
    
        if sign_delta_q_k == sign_delta_q_k1
            qf_dot = 0; % sign_delta_q_k * qc_dot_max; %0; % cambia qua per variare regola velocita
        end
    end
    qf_prec = qf_dot; % velocity continuity
    
    % trap traj
    ti = tf_prec;
    [time, q, q_dot, q_ddot, tf] = trap_qc_ddot_max_qc_dot_max(Ts, qi, qf, ti, qi_dot, qf_dot, qc_dot_max, qc_ddot_max);
    tf_prec = tf;

    % entire traj
    new_points = [new_points; [qf, tf]];
    tot_time = [tot_time, time];
    tot_q = [tot_q, q];
    tot_q_dot = [tot_q_dot, q_dot];
    tot_q_ddot = [tot_q_ddot, q_ddot];
end

traj_plotter('trap_multipoint_2', new_points, tot_time, tot_q, tot_q_dot, tot_q_ddot);
end

