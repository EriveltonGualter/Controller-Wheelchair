%{  
    Author: Erivelton Gualter 
    Date:   10/12/2016
 
    Descrition: Code to read the serial port (MPU6050) and plot the data.
%} 
          
% Clear, close and delete
delete(instrfindall);
clc, clear, close all

% Serial port object COM3 with  a baudrate 115200
arduin = serial('COM3','BaudRate',115200);

% Open comunication 
fopen(arduin);

time(1) = 0;
tf = 20;
tic;
t = 1;

a = zeros(100);
figure(1)
xlim([0 tf])
while toc <= tf  
    aux = fscanf(arduin,'%f');
    t_inst = toc;
    
    if size(aux,1) == 16
        qw(1,t) = aux(1,1); qw(2,t) = t_inst;
        qx(1,t) = aux(2,1); qx(2,t) = t_inst;
        qy(1,t) = aux(3,1); qy(2,t) = t_inst;
        qz(1,t) = aux(4,1); qz(2,t) = t_inst;
        
        euler0(1,t) = aux(5,1); euler0(2,t) = t_inst;
        euler1(1,t) = aux(6,1); euler1(2,t) = t_inst;
        euler2(1,t) = aux(7,1); euler2(2,t) = t_inst;

        ypr0(1,t) = aux(8,1); ypr0(2,t) = t_inst;
        ypr1(1,t) = aux(9,1); ypr1(2,t) = t_inst;
        ypr2(1,t) = aux(10,1); ypr2(2,t) = t_inst;
        
        aaRealx(1,t) = aux(11,1); aaRealx(2,t) = t_inst;
        aaRealy(1,t) = aux(12,1); aaRealy(2,t) = t_inst;
        aaRealz(1,t) = aux(13,1); aaRealz(2,t) = t_inst;
        
        aaWorldx(1,t) = aux(14,1); aaWorldx(2,t) = t_inst;
        aaWorldy(1,t) = aux(15,1); aaWorldy(2,t) = t_inst;
        aaWorldz(1,t) = aux(16,1); aaWorldz(2,t) = t_inst;
        
        t = t + 1;
        
    end
end
fclose(arduin); 

% Plot graphs
figure(1)
subplot(411) 
plot(qw(2,:), qw(1,:),'r')
    title('OUTPUT READABLE QUATERNION')
    xlabel('tempo (s)'); ylabel('psi');
subplot(412)
plot(qx(2,:), qx(1,:),'g')
    xlabel('tempo (s)'); ylabel('theta');
subplot(413)
plot(qy(2,:), qy(1,:),'b')
    xlabel('tempo (s)'); ylabel('phi');
subplot(414);
plot(qz(2,:), qz(1,:),'b')
    xlabel('tempo (s)'); ylabel('phi');
    
figure(2)
subplot(311) 
plot(euler0(2,:), euler0(1,:),'r')
    title('OUTPUT READABLE EULER')
    xlabel('tempo (s)'); ylabel('psi');
subplot(312)
plot(euler1(2,:), euler1(1,:),'g')
    xlabel('tempo (s)'); ylabel('theta');
subplot(313)
plot(euler2(2,:), euler2(1,:),'b')
    xlabel('tempo (s)'); ylabel('phi');
    
figure(3)
subplot(311) 
plot(ypr0(2,:), ypr0(1,:),'r')
    title('OUTPUT READABLE YAWPITCHROLL')
    xlabel('tempo (s)'); ylabel('psi');
subplot(312)
plot(ypr1(2,:), ypr1(1,:),'g')
    xlabel('tempo (s)'); ylabel('theta');
subplot(313)
plot(ypr2(2,:), ypr2(1,:),'b')
    xlabel('tempo (s)'); ylabel('phi');
    
figure(4)
subplot(311) 
plot(aaRealx(2,:), aaRealx(1,:),'r')
    title('OUTPUT READABLE REALACCEL')
    xlabel('tempo (s)'); ylabel('psi');
subplot(312)
plot(aaRealy(2,:), aaRealy(1,:),'g')
    xlabel('tempo (s)'); ylabel('theta');
subplot(313)
plot(aaRealz(2,:), aaRealz(1,:),'b')
    xlabel('tempo (s)'); ylabel('phi');
    
figure(5)
subplot(311)
plot(aaWorldx(2,:), aaWorldx(1,:),'r')
    title('OUTPUT READABLE WORLDACCEL')
    xlabel('tempo (s)'); ylabel('psi');
subplot(312)
plot(aaWorldy(2,:), aaWorldy(1,:),'g')
    xlabel('tempo (s)'); ylabel('theta');
subplot(313)
plot(aaWorldz(2,:), aaWorldz(1,:),'b')
    xlabel('tempo (s)'); ylabel('phi');
   