%% Configuracao inicial
clear
close all
clc

st = timeoptions;
st.SettleTimeThreshold = 0.05;
opt = stepDataOptions('InputOffset',1,'StepAmplitude',0.5);
format ShortEng;
eng = @(x) strtrim(evalc(sprintf('disp(%g)', x)));
%format long;

%% Definicoes
NT = 24;
ts5_prat = 55e-3;
Mp_prat = 0.1724137931;
fs = 2000;
const_polo = 1.5;
T = 1/fs;
t = 0: 0.001: 0.15;

%% Modelagem do sistema
% x1p = x2/100n
% 
% x2p = (-x1 -(NTk + 68k)x2 + 10kx3) * 1/68k*680n*NTk
% 
% x3p = u/15 - x3/0.0015

A = [0                  10e6                    0           ;
     -2.162629e-2/NT    -(68+NT)/(46.24e-3*NT)  216.262/NT  ;
     0                  0                       -666.66    ];

B = [0        ; 
     0        ;
     0.066667];

C = [1 0 0];

D = 0;

ft=ss(A, B, C, D);

%% Controlabilidade e observabilidade

if rank(ctrb(A, B)) == rank(A)
    disp('Sistema de estados controlaveis')
end

if rank(obsv(A,C)) == rank(A)
    disp('Sistema completamente observavel')
end

if rank([ctrb(A', C')'*B]') == rank(C)
    disp('Sistema de saidas controlaveis')
end

%% Identificacao do novo polo
[valy, valx] = step(ft, t, opt, st);
INFO = stepinfo(valy, valx, 'SettlingTimeThreshold',0.05);

ts5 = INFO.SettlingTime;
Mp = (INFO.SettlingMax - valy(end))/(valy(end)-valy(1));

Mp_proj = Mp/3;
Mp_ideal = Mp/2;

ts5_proj = ts5/3;
ts5_ideal = ts5/2;

%%%%%%%% Cálculo dos novos parâmetros
zeta_proj = abs(-log(Mp_proj)/(sqrt((pi^2)+(log(Mp_proj)^2))));

wn_proj = 3/(zeta_proj*ts5_proj);

%%%%%%%% Encontrar os pólos dominantes do sistema em malha fechada
s1_re = -zeta_proj*wn_proj;
s1_im = wn_proj*sqrt(1-zeta_proj^2);
s1 = complex(s1_re, s1_im);

j = [s1 conj(s1)];

%% controlador
j2 = [j const_polo*j];

A_hat_c = [A   zeros(rank(A),1);
           -C         0       ];
B_hat_c = [B ;
           0];
K = acker(A_hat_c, B_hat_c, j2);

%% Observador
Aaa = A(1,1);
Aba = A(2:end,1);
Aab = A(1,2:end);
Abb = A(2:end,2:end);
Ba  = B(1,1);
Bb  = B(2:end,1);

l = [real(1.2*const_polo*s1) real(1.2*const_polo*s1)];
Ke = acker(Abb', Aab', l)';

A_hat = Abb - Ke*Aab;
B_hat = A_hat*Ke + Aba - Ke*Aaa;
C_hat = [zeros(1,rank(A)-1);
         eye(rank(A)-1)   ];
D_hat = [ 1 ; 
         Ke];
F_hat = Bb - Ke*Ba;

%% Equação recursiva
tempo_total = 0.500;
NA = round(tempo_total/T);
r = [ones(1, NA/2) 1.5*ones(1, NA/2)];
eta_ = zeros(2, 1);
eta = zeros(2, 1);
y = zeros(1, NA);
u = zeros(1, NA);
x = zeros(3, 1);
x_planta = zeros(3, 1);
x_planta_ = zeros(3, 1);
KK = [K(1), K(2), K(3)];
Ki = -K(4);

y(1) = 0;
u(1) = 0;
xi = 0;

for n=2:(length(r))
    % Observador
    eta_ = A_hat*eta + B_hat*y(n-1) + F_hat*u(n-1);
    eta = eta + T*eta_;
    x = C_hat*eta + D_hat*y(n-1);
    
    % Controlador
    xi_ = r(n) - y(n-1);
    xi = xi + T*xi_;
    
    kx = KK*x;
    u(n) = xi*Ki - kx;
    
    % Planta
    x_planta_ = A*x_planta + B*u(n);
    x_planta = x_planta + T*x_planta_;
    y(n) = C*x_planta;
end
t_contr = 0:T:T*(length(r)-1);

y_rec = zeros(1, NA);
y_rec(1) = 0;
for n=2:(length(r))
    % Planta
    x_planta_ = A*x_planta + B*r(n);
    x_planta = x_planta + T*x_planta_;
    y_rec(n) = C*x_planta;
end
t_rec = 0:T:T*(length(r)-1);

%% Informacoes
fmin = -l(1)/(2*pi);
fprintf('\nfreq. min. amost.: %s Hz\n', eng(fmin));
fprintf('Ts5: %s s \t[%s s]\n', eng(ts5), eng(ts5_prat/2));
fprintf('Mp: %g \t\t[%g]\n', Mp, Mp_prat/2);
fprintf('Z1: %g + %gi\n', s1_re, s1_im);
fprintf('K: [%s, %s, %s]\nKi: %s\n', eng(K(1)), eng(K(2)), eng(K(3)), eng(-K(4)));

%% Escreve os resultados do projeto em um arquivo
FID = fopen('parametros.txt','w', 'n', 'UTF-8');

fprintf(FID, 'A11 = %f\r\n', A(1,1));
fprintf(FID, 'A12 = %f\r\n', A(1,2));
fprintf(FID, 'A13 = %f\r\n', A(1,3));
fprintf(FID, 'A21 = %f\r\n', A(2,1));
fprintf(FID, 'A22 = %f\r\n', A(2,2));
fprintf(FID, 'A23 = %f\r\n', A(2,3));
fprintf(FID, 'A31 = %f\r\n', A(3,1));
fprintf(FID, 'A32 = %f\r\n', A(3,2));
fprintf(FID, 'A33 = %f\r\n', A(3,3));

fprintf(FID, 'B1 = %f\r\n', B(1));
fprintf(FID, 'B2 = %f\r\n', B(2));
fprintf(FID, 'B3 = %f\r\n', B(3));

fprintf(FID, 'C1 = %f\r\n', C(1));
fprintf(FID, 'C2 = %f\r\n', C(2));
fprintf(FID, 'C3 = %f\r\n', C(3));

fprintf(FID, 'D  = %f\r\n', D);

fprintf(FID, 'Ah11 = %f\r\n', A_hat(1,1));
fprintf(FID, 'Ah12 = %f\r\n', A_hat(1,2));
fprintf(FID, 'Ah21 = %f\r\n', A_hat(2,1));
fprintf(FID, 'Ah22 = %f\r\n', A_hat(2,2));

fprintf(FID, 'Bh1 = %f\r\n', B_hat(1));
fprintf(FID, 'Bh2 = %f\r\n', B_hat(2));

fprintf(FID, 'Ch11 = %f\r\n', C_hat(1,1));
fprintf(FID, 'Ch12 = %f\r\n', C_hat(1,2));
fprintf(FID, 'Ch21 = %f\r\n', C_hat(2,1));
fprintf(FID, 'Ch22 = %f\r\n', C_hat(2,2));
fprintf(FID, 'Ch31 = %f\r\n', C_hat(3,1));
fprintf(FID, 'Ch32 = %f\r\n', C_hat(3,2));

fprintf(FID, 'Dh1 = %f\r\n', D_hat(1));
fprintf(FID, 'Dh2 = %f\r\n', D_hat(2));
fprintf(FID, 'Dh3 = %f\r\n', D_hat(3));

fprintf(FID, 'Fh1 = %f\r\n', F_hat(1));
fprintf(FID, 'Fh2 = %f\r\n', F_hat(2));

fprintf(FID, 'K1 = %f\r\n', K(1));
fprintf(FID, 'K2 = %f\r\n', K(2));
fprintf(FID, 'K3 = %f\r\n', K(3));
fprintf(FID, 'Ki = %f\r\n', -K(4));

fprintf(FID, 'fs = %f\r\n', fs);

ST1 = fclose(FID);

%% Graficos

AA_c = A_hat_c - B_hat_c * K;
BB_c = vertcat(zeros([rank(A),1]), 1);
CC_c = horzcat(C,[0]);
DD_c = D;

ft_c = ss(AA_c, BB_c, CC_c, DD_c);

% Funcao de transferencia do sistema
[yout, xout, t] = step(A,B,C,D,1,t);
[yout_c, xout_c, t_c] = step(AA_c, BB_c, CC_c, DD_c, 1, t);

figure(); plot(t, xout(:,1)), title('Estado X1');
hold on;  plot(t, xout_c(:,1)), legend('Sem controle','Com controle');
figure(); plot(t, xout(:,2)), title('Estado X2');
hold on;  plot(t, xout_c(:,2)), legend('Sem controle','Com controle');
figure(); plot(t, xout(:,3)), title('Estado X3');
hold on;  plot(t, xout_c(:,3)), legend('Sem controle','Com controle');

figure(); h = stepplot(ft, t, opt, st);
hold on;  step(ft_c, t, opt, st), legend('Sem controle','Com controle'), title('Saída do sistema');
h.showCharacteristic('SettlingTime');

ft_z = c2d(ft, T);
ft_cz = c2d(ft_c, T);
figure(); g = stepplot(ft_z, opt, st);
hold on;  step(ft_cz, opt, st), legend('Sem controle','Com controle'), title('Saída do sistema discreto');
g.showCharacteristic('SettlingTime');

%close all
figure();
plot(t_rec, y_rec, '.k');
hold on;
plot(t_contr, y, '.k');
hold on;
lsim(ft_c, r, t_contr, 'b');
hold on;
lsim(ft, r, t_contr, 'r'), title('Saída do sistema')

