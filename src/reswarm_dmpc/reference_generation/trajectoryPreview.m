init = [10.75, -8.5, 4.3];
Avx = 0.03;
Avy = -0.005;
Avz = 0.03;
f = 0.035;
dt = 0.1;

x = init(1);
y = init(2);
z = init(3);

t = 0:dt:60;
vx = Avx*cos(2*pi*f*t);
vy = Avy*ones(length(t));
vz = Avz*sin(2*pi*f*t);

for i=1:length(t)
   x = [x, x(end)+vx(i)*dt];
   y = [y, y(end)+vy(i)*dt];
   z = [z, z(end)+vz(i)*dt];
end

% Position Profile
plot3(x,y,z)

% Velocity Profile
figure()
plot(t, vx); hold on
plot(t, vy); hold on
plot(t, vz); hold off