% Threshold occupancy map into 3 states 
% occupied = > limax
% free = < limin
% unknown = between limin and limax, inclusive

function mb = binaryMap(m, limin, limax)
%     limax = 0.7;
%     limin = 0.1;
%     limin = 0.4;

    m(m > limax) = 1;
    m((m <= limax) & (m >= limin)) = 0.5;
    m(m < limin) = 0;
    mb = m;
end