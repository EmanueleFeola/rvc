function [] = trap_multipoint_1(points, Ts, qc_ddot_max)
tot_time = [];
tot_q = [];
tot_q_dot = [];
tot_q_ddot = [];

qf_prec = 0;
qc_dot = 2; % ?
for i = 1:size(points, 1)-1
    qi = points(i, 1);
    qf = points(i+1, 1);
    ti = points(i, 2);
    duration = points(i+1, 2) - ti;
    
    if i == size(points, 1)-1
        qf_dot = 0;
    end

    qi_dot = qf_prec;
    qf_dot = 0;
%     if i < size(points, 1) - 2
%         sign_delta_q_k = sign(qf - qi);
%         sign_delta_q_k1 = sign(points(i+2, 1) - qf);
%     
%         if sign_delta_q_k == sign_delta_q_k1
%             qf_dot = sign_delta_q_k * qc_dot;
%         end
%     end
    qf_prec = qf_dot; % velocity continuity

    [time, q, q_dot, q_ddot] = trap_duration_qc_ddot_max(Ts, qi, qf, ti, duration, qi_dot, qf_dot, qc_ddot_max);
    tot_time = [tot_time, time];
    tot_q = [tot_q, q];
    tot_q_dot = [tot_q_dot, q_dot];
    tot_q_ddot = [tot_q_ddot, q_ddot];
end

traj_plotter(points, tot_time, tot_q, tot_q_dot, tot_q_ddot);
end

