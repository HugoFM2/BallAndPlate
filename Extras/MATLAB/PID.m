clc; clear all; close all;


g = 9.81;
G_x = tf(-3/5*g,[1 0 0]);

Kp = -0.2;
Kd = -0.36;
Ki = -0.06;


%%
C_x = pid(Kp,Ki,Kd)


sys_cl=feedback(C_x*G_x,1);




[y,tOut,x]= step(sys_cl,15);

plot(tOut,y,LineWidth=2,DisplayName="Resposta ao Degrau");
legend()
grid
xlabel("Tempo")
ylabel("Amplitude")
title("Resposta ao degrau")


%% Simulacao simulink

% Plota px
px = out.ScopeData_x.signals.values;
tx = out.ScopeData_x.time;

plot(tx,px,LineWidth=2);
title("Posição em x da bola")
ylabel("Posição em x [m]")
xlabel("Tempo [s]")

figure
py = out.ScopeData_y.signals.values;
ty = out.ScopeData_y.time;
plot(ty,py,LineWidth=2);
title("Posição em y da bola")
ylabel("Posição em y [m]")
xlabel("Tempo [s]")

