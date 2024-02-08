function [dx] = driving_motor_model(x, u)
    
    % Params
    p = load('params.mat');
    J1 = p.J1;
    Teq_i1 = p.Teq_i1;
    Kn = p.Kn;
    Tn = p.Tn;

    % System matrix
    A=zeros(3);
    A(1,2)=1/J1;
    A(2,1)=-Kn/Teq_i1;
    A(2,2)=-1/Teq_i1;
    A(2,3)=1/(Tn*Teq_i1);
    A(3,1)=-1;

    % Control matrix
    B=zeros(3,2);
    B(1,2)=-1/J1;
    B(2,1)=Kn/Teq_i1;
    B(3,1)=1;

    % Differential state
    dx = A * x + B * u;
 
end