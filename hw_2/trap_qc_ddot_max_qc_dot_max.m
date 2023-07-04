function [t,q,q_dot,q_ddot, tf] = trap_qc_ddot_max_qc_dot_max(Ts, qi, qf, ti, qi_dot, qf_dot, qc_dot_max, qc_ddot_max)
sigma = sign(qf - qi);
if sigma == -1
    [qi, qf] = swap_variables(qi, qf);
    qi_dot = qi_dot * -1;
    qf_dot = qf_dot * -1;
end

DQ = qf - qi;

if qc_ddot_max*DQ <= abs(qi_dot^2-qf_dot^2)/2
    error('Realization constraint not satisfied: qc_ddot_max*DQ > abs(qi_dot^2-qf_dot^2)/2')
end

qDc_lim = sqrt(qc_ddot_max*DQ+(qi_dot^2+qf_dot^2)/2);
qc_dot = min(qc_dot_max, qDc_lim);

ta = (qc_dot - qi_dot)/qc_ddot_max;
td = (qc_dot - qf_dot)/qc_ddot_max;
delta_T = DQ/qc_dot + qc_dot/(2*qc_ddot_max)*((1-qi_dot/qc_dot)^2+(1-qf_dot/qc_dot)^2);

tf = ti+delta_T; % output
[t,q,q_dot,q_ddot] = trap_not_null(Ts, qi, qf, ti, tf, ta, td, qc_dot, qi_dot, qf_dot);

fprintf("qi = %.2f, qf = %.2f, ti = %.2f, tf = %.2f\n", qi, qf, ti, tf);

if sigma == -1
    q = fliplr(q);
    q_dot = -q_dot;
    q_ddot = -q_ddot;
end
end

