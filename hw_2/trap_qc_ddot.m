function [t,q,q_dot,q_ddot] = trap_qc_ddot(Ts, qi, qf, ti, tf, qc_ddot)

% check sign
qi = sign(qf - qi) * qi;
qf = sign(qf - qi) * qf;

delta = ((tf^2 - 2*ti*tf + ti^2)*qc_ddot - 4*(qf - qi)) / qc_ddot;

if delta < 0
    fprintf("traj not feasible: delta < 0\n");
    return;
end

tc = (tf-ti)/2 - (1/2) * sqrt(delta);

% call trap traj given tc
[t,q,q_dot,q_ddot] = trap_tc(Ts, qi, qf, ti, tf, tc);

end

