% MODEL: Kuka, LWR, 7DOF, standard_DH

d1 =0.4; d2 = 0.39; ee_link = 0.3;

%All link lengths and offsets are measured in m
clear L
%            theta    d           a       alpha
links = [
	    Link([0        0           0       pi/2])
		Link([0        0           0      -pi/2]) % modified from original
		Link([0        d1          0      -pi/2])
		Link([0        0           0       pi/2])
		Link([0        d2          0       pi/2])
		Link([0        0           0       -pi/2])
		Link([0        ee_link           0       0])
	];

LWR = SerialLink(links, 'name', 'Kuka LWR');

n_dofs = size(links,2);
for i=1:n_dofs
    LWR.links(i).m = 2.0;
    LWR.links(i).Jm = 0;
    LWR.links(i).G = 1;
end

qz = [0 0 0 0 0 0 0];