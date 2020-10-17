clear all;
close all;
L1 = 1; L2 = 1; L3 = 1;


p1 = [1 0 1];
p2 = [1/((2)^(1/2)),1/((2)^(1/2)),1.2];
% p2 = [0 0 3];
v_max = 1;
a_max = 10;
dt = 0.01;
q1 = InverseIK(p1);
q2 = InverseIK(p2);

ts = v_max/a_max;

tf = (q2-q1)/v_max + ts;
tspan = 0:dt:max(tf);
max_tf = max(tf);

% tspan = linspace(0,tf,N);
%position
x = ((p2(1) - p1(1))/max_tf).*tspan + p1(1);
y = ((p2(2) - p1(2))/max_tf).*tspan + p1(2);
z = ((p2(3) - p1(3))/max_tf).*tspan + p1(3);
N = length(x);
%velocities
vx = ((p2(1) - p1(1))/max_tf);
vy = ((p2(2) - p1(2))/max_tf);
vz = ((p2(3) - p1(3))/max_tf);
vel = [vx;vy;vz];

for i=1:N
    waypnts = [x(i),y(i),z(i)];
    jointConfig(i,:) = InverseIK(waypnts(1,:));
    if i ==1 || i == N 
        jointVel(i,:) = [0 0 0];
    else
        J = Jacobian_Numerical_(jointConfig(i,:));
        jointVel(i,:) = (J(1:3,:)\vel)';
    end   
end    
%for each point
n = 20;
Q = [];
Qd = [];
for i =1:N-1
    t1 = tspan(i); t2 = tspan(i+1);
    A = [1 t1 t1^2 t1^3;
         0 1 2*t1 3*t1^2;
         1 t2 t2^2 t2^3;
         0 1 2*t2 3*t2^2];
    for j = 1:3
        q1 = jointConfig(i,j); 
        q2 = jointConfig(i+1,j);
        
        
        v1 = jointVel(i,j);
        v2 = jointVel(i+1,j);
        
        c = [q1;v1;q2;v2];
        b = A\c;
        t = linspace(t1,t2,n);
        q_n(j,:) = b(1)+b(2).*t+b(3).*t.^2+b(4).*t.^3;
        qd_n(j,:) = b(2)+2*b(3).*t+3*b(4).*t.^2;
    end
    Q = [Q q_n];
    Qd = [Qd qd_n];
end
cartTraj = robotFK(Q);
veloTraj1 = diff(cartTraj(1,:)); 
% q1 = jointConfig(1,1);
% q2 = jointConfig(1,2);
% q3 = jointConfig(1,3);
% figure;
% plot(x,y'k');
% hold on;
% plot(cartTraj(1,:),cartTraj(2,:),'r');
ns = n*N-n;
figure;
plot(tspan,x);
hold on;
T = linspace(0,max_tf,ns);
plot(T,cartTraj(1,:),'k--');
grid on;
legend('X_{des}','x_{actual}');

figure;
plot(tspan,y);
hold on;
T = linspace(0,max_tf,ns);
plot(T,cartTraj(2,:),'k--');
grid on;
legend('Y_{des}','y_{actual}');

figure;
plot(tspan,z);
hold on;
T = linspace(0,max_tf,ns);
plot(T,cartTraj(3,:),'k--');
grid on;
legend('Z_{des}','Z_{actual}');


function IK = InverseIK(pnt)
x = pnt(1);
y = pnt(2);
z = pnt(3)
L1 = 1;L2 = 1; L3 = 1;
teta1 = atan2(y,x);
teta3 = acos((x^2+y^2 +(z-L1)^2 - L2^2 - L3^2)/(2*L2*L3));
m = 1;
% if teta3 > 0 
%     m = -1;
% else
%     m = 1;
% end
teta2 = -m*atan2((2*L2*sin(teta3)*L3),-(L3^2-L2^2-x^2-y^2-(z-L1)^2)) + atan2((z-L1),(x^2 + y^2)^(1/2));
IK = [teta1,teta2,teta3];
end

function J1 = Jacobian_Numerical_(q)
% syms L1 L2 L3 teta2 teta1 teta3 real
L1 = 1;
L2 = 1;
L3 = 1;
teta1 = q(1); teta2 = q(2); teta3 = q(3); 
H = Rz(teta1) * Tz(L1) * Ry(teta2) * Tx(L2) * Ry(teta3) * Tx(L3);
% H = simplify(H)
R = H(1:3,1:3);
Td = Rzd(teta1) * Tz(L1) * Ry(teta2) * Tx(L2) * Ry(teta3) * Tx(L3)* [R^-1 zeros(3,1); 0 0 0 1];%with taking Rzd
J1 = [Td(1,4),Td(2,4),Td(3,4),Td(3,2),Td(1,3),Td(2,1)]';%find element of jacobian

Td = Rz(teta1) * Tz(L1) * Ryd(teta2) * Tx(L2) * Ry(teta3) * Tx(L3) * [R^-1 zeros(3,1); 0 0 0 1];%with taking Rxd
J2 = [Td(1,4),Td(2,4),Td(3,4),Td(3,2),Td(1,3),Td(2,1)]';%find element of jacobian

Td = Rz(teta1) * Tz(L1) * Ry(teta2) * Tx(L2) * Ryd(teta3) * Tx(L3) * [R^-1 zeros(3,1); 0 0 0 1];%with taking Tyd
J3 = [Td(1,4),Td(2,4),Td(3,4),Td(3,2),Td(1,3),Td(2,1)]';%find element of jacobian

% J = [simplify(J1),simplify(J2),simplify(J3)];%find jacobian
J1 = [J1,J2,J3];
end

function FK = robotFK(Q)
%syms L1 L2 L3 teta1 teta2 teta3
L1 = 1;L2 = 1;L3 = 1; 
FK1 = [];
x = [];
y = [];
z = [];
for i = 1:length(Q)
    teta1 = Q(1,i); teta2 = Q(2,i); teta3 = Q(3,i);
    % simplify(Rz(teta1)*Tz(L1)*Ry(teta2)*Tx(L2)*Ry(teta2)*Tx(L3))
    FK1 = Rz(teta1)*Tz(L1)*Ry(-teta2)*Tx(L2)*Ry(-teta3)*Tx(L3);
    x(i) = FK1(1,4);
    y(i) = FK1(2,4);
    z(i) = FK1(3,4);
    
end
FK = [x;y;z];
end