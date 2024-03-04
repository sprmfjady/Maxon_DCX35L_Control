clear all;
clc;

% Va = 48;   
La = 0.658*10^-3;
Ra = 1.76;
J = 99.5*10^-7;
t = 3.76*10^-3;
b = J/t;
Kt = 68.3*10^-3;
Ke = Kt;
%TL = 1; % 0,0.5,1[Nm]
a = 0.72; %효율
Kg = 1/81;
Jg = 5*10^-7;
Jeq = J + Kg^2/a*Jg;
Beq = Jeq/t;

%부하 있을 때 파라미터 재설정
J_l = 0.036284;
B_l = J_l/t;
Jeq_L = Jeq+((Kg^2)*J_l/a);
Beq_L = Beq+((Kg^2)*B_l/a);

%DOB
tau=0.001;


%전류제어기
wcc = 1256.6;
Kp_c =La*wcc;
Ki_c =Ra*wcc;
Ka_c = 1/Kp_c;

%속도제어기
wcs = wcc/10;
Kp_s =Jeq_L*wcs/Kt*81;
Ki_s =Beq_L*wcs/Kt*81;
Ka_s = 1/Kp_s;

%위치제어기
wcp = wcc/100;
Kp_p =wcp;
Kd_p =wcp/wcs;

 %result = sim('PID_DOB_simul');
 %Theta_l = result.Theta_L.signals.values*(180/pi); % rad -> degree
 %Omega_l = result.Omega_L.signals.values;


% %% Draw Graph
% % 전류제어기값 결과
%  plot(result.Current.time,result.Current.signals.values,'-r')
% %  text(0.002387,0.95,'\leftarrow 95%')
%  hold on; 
%  plot(result.Current_ref.time,result.Current_ref.signals.values,'-b')
%  hold off;
%  grid on;
%  title('Current Control Graph',FontSize=20)
%  xlabel('t(sec)')
%  ylabel('Current(A)')
%  legend('Current', 'Reference')
% 
% % 속도제어기값 결과
%  plot(result.Omega_L.time,Omega_l,'-r')
% %  text(0.02387,0.95,'\leftarrow 약95%')
%  hold on; 
%  plot(result.Speed_ref.time,result.Speed_ref.signals.values,'-b')
%  hold off;
%  grid on;
%  title('Speed Control Graph',FontSize=20)
%  xlabel('t(sec)')
%  ylabel('Speed(rad/s)')
%  legend('angular velocity')

 % 위치 제어기값 결과
%  plot(result.Theta_L.time,Theta_l,'-r')
% %  text(0.2387,0.95,'\leftarrow 약95%')
%  hold on; 
%  plot(result.Angle_ref.time,result.Angle_ref.signals.values,'-b')
%  hold off;
%  grid on;
%  title('Angle Control Graph',FontSize=20)
%  xlabel('t(sec)')
%  ylabel('Angle(degree)')
%  legend('angle')



