function [t,q,q_dot,q_ddot] = trap_qc_dot_qc_ddot(Ts, qi, qf, ti, tf, qc_dot, qc_ddot)
% tf is not used because it is an output of the planner

% check sign
qi = sign(qf - qi) * qi;
qf = sign(qf - qi) * qf;
qc_dot = sign(qf - qi) * qc_dot;
qc_ddot = sign(qf - qi) * qc_ddot;

if check_triangular(qi, qf, qc_dot, qc_ddot) == 0
    % trapezoidal traj
    tc = qc_dot / qc_ddot;
    tf = (qc_dot^2 + qc_ddot * (qf - qi)) / (qc_dot * qc_ddot);
    tf = ti + tf;

    [t,q,q_dot,q_ddot] = trap_tc(Ts, qi, qf, ti, tf, tc); % given tc
else
    % triangular traj
    tc = sqrt((qf - qi) / qc_ddot);
    tf = 2*tc;
    tf = ti + tf;
    qc_dot = (qf - qi) / tc;

    [t,q,q_dot,q_ddot] = trap_qc_dot(Ts, qi, qf, ti, tf, qc_dot); % given qc_dot
end

end

