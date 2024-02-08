% Parameters
J = 0.022 ;
Ts =1e-3;
Teq_i=sqrt(2)*2*Ts;
Kn=J/2/Teq_i;
%Tn=Teq_i;
Tn=20*Teq_i;

%Matrix
A=zeros(3);
A(1,2)=1/J;
A(2,1)=-Kn/Teq_i;
A(2,2)=-1/Teq_i;
A(2,3)=1/(Tn*Teq_i);
A(3,1)=-1;

B=zeros(3,2);
B(1,2)=-1/J;
B(2,1)=Kn/Teq_i;
B(3,1)=1;

%Simulation
N=10000;
u=zeros(2,N);
u(1,:)=100*ones(1,N);
u(2,N/2:N)=50*ones(1,N/2+1);

x=zeros(3,N+1);
x_dot=zeros(3,N);
dt=0.1e-3;

for n =1:N
  x_dot=A*x(:,n)+B*u(:,n);
  x(:,n+1)=x_dot*dt+x(:,n);
end

figure(1)
plot(1:N,u)
hold on
plot(1:N+1,x)
hold off