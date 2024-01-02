clc; clear all; close all;


tabela = readtable("OUT BOM 3.csv");


%% Primeira Tabela
clc
t = tabela.time;
t = t(tabela.time>2990 & tabela.time<3020);
t = t-2990;


x = round(tabela.x,2);
x = x(tabela.time>2990 & tabela.time<3020);

y = round(tabela.y,2);
y = y(tabela.time>2990 & tabela.time<3020);

plot(t,x,Color="k",LineWidth=1.5,DisplayName="Posição em x"); hold on;

plot(t,y,Color="blue",LineWidth=1.5,DisplayName="Posição em y"); hold on;
xlabel("Tempo [s]")
ylabel("Posição [cm]")
title("Posição da bola")
legend();


%% Segunda Tabela

clc
t = tabela.time;
t = t(tabela.time>3489 & tabela.time<3515);
t = t-3489;


x = round(tabela.x,2);
x = x(tabela.time>3489 & tabela.time<3515);

y = round(tabela.y,2);
y = y(tabela.time>3489 & tabela.time<3515);

plot(t,x,Color="k",LineWidth=1.5,DisplayName="Posição em x"); hold on;

plot(t,y,Color="blue",LineWidth=1.5,DisplayName="Posição em y"); hold on;
xlabel("Tempo [s]")
ylabel("Posição [cm]")
title("Posição da bola")
legend();

