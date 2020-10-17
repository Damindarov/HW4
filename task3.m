q10 = 0; q1f = 2; v1 = 1; a1=10;
q20 = 0; q2f = 3; v2 = 1; a2=10;
q30 = 0; q3f = 4; v3 = 1; a3=10;
delta_t = 0.01;
n = 0;
v10 = 0; v20 = 0; v30 = 0;
while(floor(delta_t*10^n)~=delta_t*10^n)
    n=n+1;
end
E = 1*10^-n;
t1a = v1/a1;
if rem(t1a,delta_t)~=0
    t1a_new = round(t1a,n)+E;
else
    t1a_new = round(t1a,n);
end
t1f = (q1f - q10)/v1 + t1a_new;
if rem(t1f,delta_t)~=0
    t1f_new = round(t1f,n) + E;
else
    t1f_new = round(t1f,n);
end

t2a = v2/a2;
if rem(t2a,delta_t)~=0
    t2a_new = round(t2a,n)+E;
else
    t2a_new = round(t2a,n);
end

t2f = (q2f - q20)/v2 + t2a_new;
if rem(t2f,delta_t)~=0
    t2f_new = round(t2f,n)+E;
else
    t2f_new = round(t2f,n);
end

t3a = v3/a3;
if rem(t3a,delta_t)~=0
    t3a_new = round(t3a,n)+E;
else
    t3a_new = round(t3a,n);
end

t3f = (q3f - q30)/v3 + t3a_new;
if rem(t3f,delta_t)~=0
    t3f_new = round(t3f,n)+E;
else
    t3f_new = round(t3f,n);
end

tf_max = max([t1f_new t2f_new t3f_new]);

if t2f_new == tf_max
    tf_new = t2f_new;
    ta_new = t2a_new;
end
if t3f_new == tf_max
    tf_new = t3f_new;
    ta_new = t3a_new;
end
if t1f_new == tf_max
    tf_new = t1f_new;
    ta_new = t1a_new;
end

v1_new = ((q1f - q10)/(tf_new-ta_new));
a1_new = v1_new/ta_new;

v2_new = ((q2f - q20)/(tf_new-ta_new));
a2_new = v2_new/ta_new;

v3_new = ((q3f - q30)/(tf_new-ta_new));
a3_new = v3_new/ta_new;

% joint1 t0 ->ta
a10 = q10;
a11 = v10;
a12 = 0.5*a1_new;
% ta->tf-ta
a20 = q10 + 0.5*a1_new*ta_new^2 - v1_new*ta_new;
a21 = v1_new;
% tf-ta ->tf
a30 = q1f - 0.5*a1_new*tf_new^2;
a31 = a1_new*tf_new;
a32 = -0.5*a1_new;
% joint1 t0 ->ta
b10 = q20;
b11 = v20;
b12 = 0.5*a2_new;
% ta->tf-ta
b20 = q20 + 0.5*a2_new*ta_new^2 - v2_new*ta_new;
b21 = v2_new;
% tf-ta ->tf
b30 = q2f - 0.5*a2_new*tf_new^2;
b31 = a2_new*tf_new;
b32 = -0.5*a2_new;
% joint3 t0->ta
c10 = q30;
c11 = v30;
c12 = 0.5*a3_new;
% ta->tf-ta
c20 = q30 + 0.5*a3_new*ta_new^2 - v3_new*ta_new;
c21 = v3_new;
% tf-ta ->tf
c30 = q3f - 0.5*a3_new*tf_new^2;
c31 = a3_new*tf_new;
c32 = -0.5*a3_new;

t = 0:delta_t:tf_new;

q1 = (a10 +a11.*t+a12.*t.^2).*(t<=ta_new)...
    +(a20+a21.*t).*(t>ta_new).*(t <= (tf_new-ta_new))...
    +(a30+a31.*t+a32.*t.^2).*(t>(tf_new-ta_new)).*(t<=tf_new);

q2 = (b10 +b11.*t+b12.*t.^2).*(t<=ta_new)...
    +(b20+b21.*t).*(t>ta_new).*(t <= (tf_new-ta_new))...
    +(b30+b31.*t+b32.*t.^2).*(t>(tf_new-ta_new)).*(t<=tf_new);

q3 = (c10 +c11.*t+c12.*t.^2).*(t<=ta_new)...
    +(c20+c21.*t).*(t>ta_new).*(t <= (tf_new-ta_new))...
    +(c30+c31.*t+c32.*t.^2).*(t>(tf_new-ta_new)).*(t<=tf_new);
v1 = (a11 + 2*a12.*t).*(t<=ta_new)...
     +(a21).*(t>ta_new).*(t <= (tf_new-ta_new))...
     +(a31+2*a32.*t).*(t>(tf_new-ta_new)).*(t<=tf_new);
v2 = (b11 + 2*b12.*t).*(t<=ta_new)...
     +(b21).*(t>ta_new).*(t <= (tf_new-ta_new))...
     +(b31+2*b32.*t).*(t>(tf_new-ta_new)).*(t<=tf_new);
v3 = (c11 + 2*c12.*t).*(t<=ta_new)...
     +(c21).*(t>ta_new).*(t <= (tf_new-ta_new))...
     +(c31+2*c32.*t).*(t>(tf_new-ta_new)).*(t<=tf_new); 
acc1 = (2*a12).*(t<=ta_new)...
    +(0).*(t>ta_new).*(t <= (tf_new-ta_new))...
    +(2*a32).*(t>(tf_new-ta_new)).*(t<=tf_new);
acc2 = (2*b12).*(t<=ta_new)...
    +(0).*(t>ta_new).*(t <= (tf_new-ta_new))...
    +(2*b32).*(t>(tf_new-ta_new)).*(t<=tf_new);
acc3 = (2*c12).*(t<=ta_new)...
    +(0).*(t>ta_new).*(t <= (tf_new-ta_new))...
    +(2*c32).*(t>(tf_new-ta_new)).*(t<=tf_new);
figure
plot(t,q1,'red','linewidth',2);
hold on;
plot(t,q2,'green','linewidth',2);
plot(t,q3,'blue','linewidth',2);
hold on
title('position vs time')

figure
plot(t,v1,'r','linewidth',2)
hold on
plot(t,v2,'g','linewidth',2)
plot(t,v3,'b','linewidth',2)
title('V vs time')

figure
plot(t,acc1,'r','linewidth',2)
hold on
plot(t,acc2,'g','linewidth',2)
plot(t,acc3,'b','linewidth',2)
title('a vs time')