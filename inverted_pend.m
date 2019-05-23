
clear all;
clc;

M = 0.5; %mass of the cart
m = 0.2; %mass of the pendulum
b = 0.1; %coefficient of the friction of the cart
l = 0.3; %lenght to the pendulum centre of mass
i = 0.006; %mass moment of inertia of the pendulum
g = 9.8; %acceleration due to gravity
  

%% linearization of the state space model
den = i*(M+m)+M*m*l^2; %denominator for the A and B matricies
A = [0 1  0  0; 0 -(i+m*l^2)*b/den  (m^2*g*l^2)/den  0;0 0  0 1;0 -(m*l*b)/den  m*g*l*(M+m)/den  0]
B = [ 0; (i+m*l^2)/den;0;m*l/den]
C = [1 0 0 0;0 0 1 0]
D = [0;0]
linsys_cont = ss(A,B,C,D);

%% Continuous state space to discrete time model
T = 0.01; %sampling time
dissys = c2d(linsys_cont,T);
Ad = dissys.A;
Bd = dissys.B;
Cd = dissys.C;
Dd = dissys.D;

%% To check controllability and Observability

Co = ctrb(Ad,Bd);
unco = length(Ad)-rank(Co);

if unco == 0
    disp('the system is controllable')
else
    disp('the system is not controllable')
end

Ob = obsv(Ad,Cd);
unob = length(Ad)-rank(Ob);

if unob == 0
    disp('the system is obsevable')
else
    disp('the system is not observable')
end

%% To find the control gain through Pole placement method
 Closed_pole = [0.7 0.99 0.75 0.96];
 k = place(Ad,Bd,Closed_pole);
 disp(k)
 
 %% To find the control gain using LQR method
 Q = [100 0 0 0;0 0.2 0 0;0 0 100 0; 0 0 0 10];
R = 0.1;
ko = lqr(Ad,Bd,Q,R);
disp(ko)

%% Simulate of the LTI plant based on control gain from pole placement method
As = (Ad-Bd*k);
Bs = Bd;
Cs = Cd;
Ds = Dd;
t = 0:0.1:10;
u = 0.2*ones(size(t));
clsys = ss(As,Bs,Cs,Ds);
xo = [0 0 0.05 0]';
[y,t] = lsim(clsys,u,t,xo);
figure(1)
[Axis,H1] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(Axis(1),'Ylabel'),'String','position')
set(get(Axis(2),'Ylabel'),'String','angle')
title('pole placement method')
hold on
%% Simulate the LTI plant based on control gain from LQR method
Aop = (Ad-Bd*ko);
Bop = Bd;
Cop = Cd;
Dop = Dd;
t1 = 0:0.1:10;
u1 = 0.2*ones(size(t1));
lqrsys = ss(Aop,Bop,Cop,Dop);
xo = [0 0 0.05 0]';
[y1,t1] = lsim(lqrsys,u1,t1,xo);
figure(2)
[Axis1,H1] = plotyy(t1,y1(:,1),t1,y1(:,2),'plot');
set(get(Axis1(1),'Ylabel'),'String','position')
set(get(Axis1(2),'Ylabel'),'String','angle')
title('LQR method')



