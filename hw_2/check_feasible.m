function [out] = check_feasible(ti, tf, tc)
    out = tc > (tf-ti)/2;
end

