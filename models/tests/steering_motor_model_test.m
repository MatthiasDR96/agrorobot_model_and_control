% Parameters
J = 0.022;
Ts = 1e-3;
Teq_i = sqrt(2)*2*Ts;
Kn1 = J/2/Teq_i;
Tn1 = 20*Teq_i;
Kn2 = J/2/Teq_i;
Tn2 = 20*Teq_i;

% Matrix
A=zeros(5);
A(1,2)=1;
A(2,3)=1/J;
A(3,1)=-Kn1*Kn2/Teq_i;
A(3,2)=-Kn1/Teq_i;
A(3,3)=-1/Teq_i;
A(3,4)=1/(Tn1*Teq_i);
A(3,5)=Kn1/(Tn2*Teq_i);
A(4,1)=-Kn2;
A(4,2)=-1;
A(4,5)=1/Tn2;
A(5,1) = -1;
A

B=zeros(5,2);
B(2,2)=-1/J;
B(3,1)=Kn1*Kn2/Teq_i;
B(4,1)=Kn2;
B(5,1)=1;
B

%Simulation
N=10000;
u=zeros(2,N);
u(1,:)= pi/18 * ones(1,N);
u(2,:)=10*ones(1,N);

x=zeros(5,N+1);
x_dot=zeros(5,N);
dt=0.001;

for n =1:N
  x_dot=A*x(:,n)+B*u(:,n);
  x(:,n+1)=x_dot*dt+x(:,n);
end

figure(1)
subplot(221)
plot(1:N,u(1,:))
hold on
plot(1:N,x(1,1:end-1))
legend(["Reference angle", "Real angle"])
subplot(222)
plot(1:N,x(2,1:end-1))
legend('Motor speed')
subplot(223)
plot(1:N,u(2,:))
hold on
plot(1:N,x(3,1:end-1))
legend(["Disturbance torque", "Motor torque"])