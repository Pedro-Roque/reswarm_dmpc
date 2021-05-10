init = [0.0, 0.0, 0.0];
A = 0.06;
f = 0.035;
dt = 0.5;

x = init(1);
y = init(2);
z = init(3);

t = 0:dt:120;
vx = A*cos(2*pi*f*t);
vy = -0.005*ones(length(t));
vz = A*sin(2*pi*f*t);

for i=1:length(t)
   x = [x, x(end)+vx(i)*dt];
   y = [y, y(end)+vy(i)*dt];
   z = [z, z(end)+vz(i)*dt];
end

plot3(x,y,z)