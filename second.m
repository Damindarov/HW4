Q_matrix1 = [0 0 0 2 0 0];
Q_matrix2 = [0 0 0 3 0 0];
Q_matrix3 = [0 0 0 4 0 0];
q1 = (matrix_A(2)^(-1))*Q_matrix1'
q2 = (matrix_A(2)^(-1))*Q_matrix2'
q3= (matrix_A(2)^(-1))*Q_matrix3'
t = 0:0.1:2;
Q1 = q1(1) + q1(2)*t + q1(3).*t.^2 + q1(4).*t.^3 + q1(5).*t.^4 + q1(6).*t.^5;
Q2 = q2(1) + q2(2)*t + q2(3).*t.^2 + q2(4).*t.^3 + q2(5).*t.^4 + q2(6).*t.^5;
Q3 = q3(1) + q3(2)*t + q3(3).*t.^2 + q3(4).*t.^3 + q3(5).*t.^4 + q3(6).*t.^5;

v1 = diff(Q1);
v2 = diff(Q2);
v3 = diff(Q3);

a1 = diff(v1);
a2 = diff(v2);
a3 = diff(v3);

figure;
plot(t,Q1,'r');
hold on;
plot(t,Q2,'g');
plot(t,Q3,'b');
legend('q1','q2','q3')

figure;
plot(t(2:length(t)),v1,'r');
hold on;
plot(t(2:length(t)),v2,'g');
plot(t(2:length(t)),v3,'b');
legend('Vq1','Vq2','Vq3')

figure;
plot(t(3:length(t)),a1,'r');
hold on;
plot(t(3:length(t)),a2,'g');
plot(t(3:length(t)),a3,'b');
legend('a q1','a q2','a q3')


function A = matrix_A(t)
A = [1 0 0 0 0 0; 
     0 1 0 0 0 0;
     0 0 2 0 0 0;
     1 t t^2 t^3 t^4 t^5;
     0 1 2*t 3*t^2 4*t^3 5*t^4;
     0 0 2 6*t 12*t^2 20*t^3];
end