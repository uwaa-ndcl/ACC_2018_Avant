% load some constants into the workspace
const = params();
g = const.g;
mc = const.mc;
ma = const.ma;
mb = const.mb;
mr = const.mr;
mt = mc + 4*ma + 4*mb + 4*mr;
c1 = const.c1;
c2 = const.c2;
l = const.l;

% min, max, and middle extension lengths
dmin = 1*l;    % physical limit of arms
dmax = 2*l;
dmid = 1.5*l;