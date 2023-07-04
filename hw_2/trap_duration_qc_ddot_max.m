function [t,q,q_dot,q_ddot, qc_dot] = trap_duration_qc_ddot_max(Ts, qi, qf, ti, delta_T, qi_dot, qf_dot, qc_ddot_max)
fprintf("qi = %2.f, qf = %2.f\n", qi, qf);

% check sign
sigma = sign(qf - qi);
if sigma == -1
    [qi, qf] = swap_variables(qi, qf);
    qi_dot = qi_dot * -1;
    qf_dot = qf_dot * -1;
end

DQ = qf - qi;

if qc_ddot_max*(qf - qi) <= abs(qi_dot^2-qf_dot^2)/2
    error('Realization constraint not satisfied: qc_ddot_max*DQ > abs(qi_dot^2-qf_dot^2)/2')
end

qc_ddot_lim = 2*(qf - qi)-(qi_dot+qf_dot)*delta_T+sqrt(4*(qf - qi)^2-4*(qf - qi)*(qi_dot+qf_dot)*delta_T+2*(qi_dot^2+qf_dot^2)*delta_T^2);

qc_ddot_lim = qc_ddot_lim / (delta_T^2);

if qc_ddot_max <= qc_ddot_lim
    error('Maximum acceleration must be bigger than %.3f', qc_ddot_lim)
end

DDQ = qf_dot - qi_dot;
d = qc_ddot_max^2*delta_T^2-4*qc_ddot_max*DQ+2*qc_ddot_max*(qi_dot+qf_dot)*delta_T-DDQ^2;
qc_dot = (qi_dot + qf_dot + qc_ddot_max*delta_T - sqrt(d))/2;
ta = (qc_dot - qi_dot)/qc_ddot_max;
td = (qc_dot - qf_dot)/qc_ddot_max;

[t,q,q_dot,q_ddot] = trap_not_null(Ts, qi, qf, ti, ti+delta_T, ta, td, qc_dot, qi_dot, qf_dot);

if sigma == -1
    q = fliplr(q);
    q_dot = -q_dot;
    q_ddot = -q_ddot;
end
end

