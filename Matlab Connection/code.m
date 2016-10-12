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
    
    if size(aux,1) == 3
        x(1,t) = aux(1,1); x(2,t) = t_inst;
        y(1,t) = aux(2,1); y(2,t) = t_inst;
        z(1,t) = aux(3,1); z(2,t) = t_inst;
        
        t = t + 1;
    end
end
fclose(arduin); 

% Plot graphs
title('Leitura do MPU6050')
subplot(311) 
title('Leitura do MPU6050')
plot(x(2,:), x(1,:),'r')
    xlabel('tempo (s)'); ylabel('psi');
subplot(312)
plot(y(2,:), y(1,:),'g')
    xlabel('tempo (s)'); ylabel('theta');
subplot(313)
plot(z(2,:), z(1,:),'b')
    xlabel('tempo (s)'); ylabel('phi');
    
save('data');

