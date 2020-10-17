J = Jacobian_Numerical_()

function J = Jacobian_Numerical_()
syms L1 L2 L3 teta2 teta1 teta3 real
% L1 = 1;
% L2 = 1;
% L3 = 1;
% teta1 = 0; teta2 = 0; teta3 = 0; 
H = Rz(teta1) * Tz(L1) * Ry(teta2) * Tx(L2) * Ry(teta3) * Tx(L3);
H = simplify(H);
R = H(1:3,1:3);
Td = Rzd(teta1) * Tz(L1) * Ry(teta2) * Tx(L2) * Ry(teta3) * Tx(L3)* [R^-1 zeros(3,1); 0 0 0 1];%with taking Rzd
J1 = [Td(1,4),Td(2,4),Td(3,4),Td(3,2),Td(1,3),Td(2,1)]';%find element of jacobian

Td = Rz(teta1) * Tz(L1) * Ryd(teta2) * Tx(L2) * Ry(teta3) * Tx(L3) * [R^-1 zeros(3,1); 0 0 0 1];%with taking Rxd
J2 = [Td(1,4),Td(2,4),Td(3,4),Td(3,2),Td(1,3),Td(2,1)]';%find element of jacobian

Td = Rz(teta1) * Tz(L1) * Ry(teta2) * Tx(L2) * Ryd(teta3) * Tx(L3) * [R^-1 zeros(3,1); 0 0 0 1];%with taking Tyd
J3 = [Td(1,4),Td(2,4),Td(3,4),Td(3,2),Td(1,3),Td(2,1)]';%find element of jacobian

J = [simplify(J1),simplify(J2),simplify(J3)]%find jacobian
% J = [J1,J2,J3];
end
