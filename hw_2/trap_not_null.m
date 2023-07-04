function [t,q,q_dot,q_ddot] = trap_not_null(Ts, qi, qf, ti, tf, ta, td, qc_dot, qi_dot, qf_dot)
% generic formula (not-null initial velocities, ta != tc)
% formula from L3-Point2Point_Trapezoidal, slide 17-18-19

% flip signs if needed
% sigma = sign(qf - qi);
% qi = sigma * qi;
% qf = sigma * qf;
% qi_dot = sigma * qi_dot;
% qf_dot = sigma * qf_dot;
% qc_dot = sigma * qc_dot;

t=ti:Ts:tf; % total time vector

%% acceleration phase (A)
syms t_sym
time_A = [ti:Ts:ti+ta-Ts];
qA = qi + qi_dot*(t_sym-ti)+((qc_dot-qi_dot)/(2*ta))*(t_sym-ti).^2;
qA_dot = diff(qA, t_sym);
qA_ddot = diff(qA_dot, t_sym);

qA = subs(qA, t_sym, time_A);
qA_dot = subs(qA_dot, t_sym, time_A);
qA_ddot = subs(qA_ddot, t_sym, time_A);

%% cruise velocity phase (B)
time_B = [ti+ta:Ts:tf-td-Ts];
qB = qi + qi_dot*ta/2+qc_dot*(t_sym-ti-ta/2);
qB_dot = diff(qB, t_sym);
qB_ddot = diff(qB_dot, t_sym);

qB = subs(qB, t_sym, time_B);
qB_dot = subs(qB_dot, t_sym, time_B);
qB_ddot = subs(qB_ddot, t_sym, time_B);

%% deceleration phase (C)
% time_C = [tf-tc:Ts:tf]; % no buono, cosi possono rimanere dei buchi
s=size(t,2)-size(time_A,2)-size(time_B,2);
time_C=linspace(tf-td,tf,s); 
qC = qf-qf_dot*(tf-t_sym)-((qc_dot-qf_dot)/(2*td))*(tf-t_sym).^2;
qC_dot = diff(qC, t_sym);
qC_ddot = diff(qC_dot, t_sym);

qC = subs(qC, t_sym, time_C);
qC_dot = subs(qC_dot, t_sym, time_C);
qC_ddot = subs(qC_ddot, t_sym, time_C);

%% traj
q = [qA, qB, qC];
q_dot = [qA_dot, qB_dot, qC_dot];
q_ddot = [qA_ddot, qB_ddot, qC_ddot];

% flop signs if needed
% q = sigma * q;
% q_dot = sigma * q_dot;
% q_ddot = sigma * q_ddot;

end

