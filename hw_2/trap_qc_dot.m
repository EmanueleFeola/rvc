function [t,q,q_dot,q_ddot] = trap_qc_dot(Ts, qi, qf, ti, tf, qc_dot)
% sigma = sign(qf - qi);
% if sigma == -1
%     [qi, qf] = swap_variables(qi, qf);
% end

tc = (tf-ti) - (qf - qi) / qc_dot;

% sanity check
if tc < 0 || tc > (tf - tc)
    fprintf("traj not feasible: tc < 0 || tc > (tf - tc)\n");
    return;
end

% call trap traj given tc
[t,q,q_dot,q_ddot] = trap_tc(Ts, qi, qf, ti, tf, tc);

end

