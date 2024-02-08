function [dx] = steering_motor_model(x, u)
    
    % Parameters
    p = load('params.mat');
    J2 = p.J2;
    Teq_i2 = p.Teq_i2;
    Kn1 = p.Kn1;
    Tn1 = p.Tn1;
    Kn2 = p.Kn2;
    Tn2 = p.Tn2;
    
    % System matrix
    A=zeros(5);
    A(1,2)=1;
    A(2,3)=1/J2;
    A(3,1)=-Kn1*Kn2/Teq_i2;
    A(3,2)=-Kn1/Teq_i2;
    A(3,3)=-1/Teq_i2;
    A(3,4)=1/(Tn1*Teq_i2);
    A(3,5)=Kn1/(Tn2*Teq_i2);
    A(4,1)=-Kn2;
    A(4,2)=-1;
    A(4,5)=1/Tn2;
    A(5,1) = -1;

    % Control matrix
    B=zeros(5,2);
    B(2,2)=-1/J2;
    B(3,1)=Kn1*Kn2/Teq_i2;
    B(4,1)=Kn2;
    B(5,1)=1;
    
    % Differential state
    dx = A * x + B * u;
 
end