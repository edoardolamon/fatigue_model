% Model
l1 = Link('d', 0, 'a', 1, 'alpha', 0);
l2 = Link('d', 0, 'a', 1, 'alpha', 0);
l1.m = 1;
l2.m = 1;
planar_arm = SerialLink([l1 l2], 'name', '2 DoFs Planar Arm');
planar_arm.qlim =  [-3/4*pi*ones(2,1)  3/4*pi*ones(2,1)];

g = [0; 9.81; 0]; 
step = 0.1;
[Q1, Q2] = meshgrid(planar_arm.qlim(1,1):step:planar_arm.qlim(1,2), planar_arm.qlim(2,1):step:planar_arm.qlim(2,2));
tau_grav = zeros(size(Q1));

for (j=1:size(Q1,2))
    for (i=1:size(Q1,2))
        tmp = planar_arm.gravload([Q1(i,j) Q2(i,j)],g);
        tau_grav(i,j) = norm(tmp);
    end
end

surf(Q1, Q2, tau_grav)

