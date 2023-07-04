function [out] = check_triangular(qi, qf, qc_dot, qc_ddot)
    out = (qf - qi) < (qc_dot^2) / qc_ddot;
end

