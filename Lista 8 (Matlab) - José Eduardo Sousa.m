% Lista 8 - Produtos Diferenciados - Microeconomia III (2018)
% Aluno: Jos� Eduardo Sousa

clear clear all clc

% Definindo o diret�rio de trabalho:
cd('C:\Users\joseg_000.PC-JE\Google Drive\FGV-EPGE\Microeconomia\Micro 3\Listas de Exerc�cios\Minhas Resolu��es\Lista 8 - Micro 3\data')

% Importando os dados:
data = readtable('data.txt')


%% Item 3

% Vamos agora organizar a base de dados para que possamos rodar a regress�o
% OLS do modelo descrito no item 2.

% Para isso, vamos empilhar o modelo e criar as vari�veis necess�rias:

% Empilhando as vari�veis market-share, pre�o e canais:
market_share = [data.s1 ; data.s2]
preco = [data.p1 ; data.p2]
canais = [data.x1 ; data.x2]
z = [data.z1; data.z2]

% Market-Share da outside option, e empilhando:
S0 = [ (1 - data.s1 - data.s2) ; (1 - data.s1 - data.s2) ]

% Criando a vari�vel dependente:
diff_log_mkt_share = log(market_share) - log(S0)
% ou
diff_log_mkt_share = log(market_share ./ S0)

% Criando uma vari�veis dummy para diferenciar o intercepto entre as firmas 1 e 2:

% Dummy de da firma C�u:
% Se j = 1, a dummy ceu assume valor ceu = 1
% Se j = 2, a dummy ceu assume valor ceu = 0, pois se trata da firma "rede"
ceu = [ ones(500, 1) ; zeros(500, 1)]

% Dummy de da firma Rede:
% Se j = 1, a dummy rede assume valor rede = 0, , pois se trata da firma "C�u"
% Se j = 2, a dummy ceu assume valor rede = 1
rede = [ zeros(500, 1) ; ones(500, 1)]

% Adicionando todas as vari�veis necess�rias para a regress�o em uma �nica base de dados empilhada:
dados = [diff_log_mkt_share ones(1000, 1) ceu rede canais preco]

Y = diff_log_mkt_share                      % Vetor da vari�vel dependente
X = [ones(1000, 1) ceu canais preco]        % Matriz de Regressores

% Regress�o OLS de diff_log_mkt_share em ceu, canais e pre�os, com constante:
coeficientes_ols = regress(Y, X)

% A regress�o OLS nos deu os seguintes coeficientes estimados:

% (Intercepto) -2.0664      --->    alpha1 = Intercepto + ceu = -1.7545
% ceu           0.3119      --->    alpha2 = Intercepto = -2.0664
% canais        0.0040      --->    gamma  = canais = 0.0040
% preco         0.0007      --->    - beta = lambda = preco = 0.0007
%                           --->      beta = - 0.0007



% A regress�o tamb�m poderia ter sido feita com uma dummy para cada firma, e sem constante:

Y = diff_log_mkt_share                         % Vetor da vari�vel dependente
X_alternativo = [ceu rede canais preco]        % Matriz de Regressores

% Regress�o OLS de diff_log_mkt_share em ceu, canais e pre�os, com constante:
coeficientes_ols_alternativo = regress(Y, X_alternativo)

% A regress�o OLS nos deu os seguintes coeficientes estimados:

%   alpha1 = -1.7545   (Intercepto Firma 1 - ceu)
%   alpha2 = -2.0664   (Intercepto Firma 2 - rede)
%   gamma  = 0.0040    (Canais)
%   - beta = lambda = preco = 0.0007 ------->>  beta = - 0.0007 (Pre�o)


%% Item 3 (De outra Maneira - Seguindo a dica do enunciado):

% A vari�vel dependente � constru�da da mesma maneira

% Vetor de Regressores:

X_dica = [ ones(500, 1) zeros(500, 1) data.x1 data.p1 ; zeros(500, 1) ones(500, 1) data.x2 data.p2 ]

% Note que a diferen�a entre esta matriz de regressores e a feita anteriormente � a seguinte:
%   - Em X h� um intercepto (grupo base "Rede") e uma dummy para a firma "ceu"
%   - Em X_dica n�o h� intercepto, e h� uma dummy para cada firma, ceu (coluna 1) e rede (coluna 2).
%   - As colunas 3 e 4 que representam as variaveis x (canal) e p (pre�o) s�o exatamente as mesmas.


% Regress�o:

coeficientes_ols_2 = regress(Y, X_dica)

% A regress�o OLS nos deu os seguintes coeficientes estimados:

%   alpha1 = -1.7545   (Intercepto Firma 1 - ceu)
%   alpha2 = -2.0664   (Intercepto Firma 2 - rede)
%   gamma  = 0.0040    (Canais)
%   - beta = lambda = preco = 0.0007 ------->>  beta = - 0.0007 (Pre�o)


%% Item 6:

% Para corrigir o problema de endogeneidade do modelo anterior, iremos usar 
% a vari�vel z (estimativa do custo marginal) como instrumento, e faremos a
% estima��o pelo m�todo de M�nimos Quadrados em 2 Est�gios (MQ2E):

% Vari�vel end�gena: preco
P = preco

% Intrumento:
z = [data.z1 ; data.z2]

% Matriz de Regressores ex�genos e instrumentos:
Z = [ ones(500, 1) zeros(500, 1) data.x1 data.z1 ; zeros(500, 1) ones(500, 1) data.x2 data.z2 ]

% Outra forma de montar a matriz Z, usando as vari�veis empilhadas criadas antes:
Z = [ceu, rede, canais, z]

% Primeiro Est�gio: Regress�o da vari�vel end�gena nas vari�veis ex�genas e nos instrumentos (P em Z):
coef_estagio = regress(P, Z)

% Computando o valor ajustado de P (proje��o de P em Z):
P_hat = Z * coef_estagio


% Segundo est�gio: Reestimamos a equa��o original, mas substituindo a vari�vel end�gena P por P_hat:

X_vi = [ceu rede canais P_hat]       % Nova matriz de regressores.

coef_2SLS = regress(Y, X_vi)


% A regress�o MQ2E nos deu os seguintes coeficientes estimados:

%   alpha1 = 0.6043   (Intercepto Firma 1 - ceu)
%   alpha2 = 0.2140   (Intercepto Firma 2 - rede)
%   gamma  = 0.0048    (Canais)
%   - beta = lambda = preco = -0.0157 ------->>  beta = 0.0157 (Pre�o)



%% Item 7: (Acho que est� maneira est� incorreta - ver pr�ximo t�pico)
% 
% % Computando os par�metros estimados no item anterior:
% alpha1_hat = coef_2SLS(1)
% alpha2_hat = coef_2SLS(2)
% gamma_hat  = coef_2SLS(3)
% lambda_hat = coef_2SLS(4)
% beta_hat   = - lambda_hat
% 
% % Calculando o Market-Share m�dio, com choque nulo:
% v1 = exp( alpha1_hat + gamma_hat * mean(data.x1) - beta_hat * mean(data.p1) )
% v2 = exp( alpha2_hat + gamma_hat * mean(data.x2) - beta_hat * mean(data.p2) )
% 
% s1_medio = v1/(1 + v1 + v2)
% s2_medio = v2/(1 + v1 + v2)
% 
% % Calculando agora o custo marginal m�dio estimado:
% cmg1 = mean(data.p1) + 1/(beta_hat * (s1_medio - 1))
% cmg2 = mean(data.p2) + 1/(beta_hat * (s2_medio - 1))
% 
% % Custo marginal m�dio da firma 1 (ceu)  = 70.5032
% % Custo marginal m�dio da firma 2 (rede) = 70.3510
% 
% 
% 
% % Outra maneira de calcular o market-share m�dio, calculando o market-share
% % estimado para cada t, e depois tirando a m�dia:
% v1_t = exp( alpha1_hat + gamma_hat * data.x1 - beta_hat * data.p1 )
% v2_t = exp( alpha2_hat + gamma_hat * data.x2 - beta_hat * data.p2 )
% 
% s1_t = v1_t ./(1 + v1_t + v2_t)
% s2_t = v2_t ./(1 + v1_t + v2_t)
% 
% s1_MEDIO = mean(s1_t)
% s2_MEDIO = mean(s2_t)
% 
% % Calculando agora o custo marginal m�dio estimado:
% cmg1_ = mean(data.p1) + 1/(beta_hat * (s1_MEDIO - 1))
% cmg2_ = mean(data.p2) + 1/(beta_hat * (s2_MEDIO - 1))
% 
% cmg = [cmg1 cmg2]
% 
% % Custo marginal m�dio da firma 1 (ceu)  = 70.4505
% % Custo marginal m�dio da firma 2 (rede) = 70.3079


%% Item 7 (Outra Maneira - Creio que mais correta)

% Do jeito que foi feito o item 7 anteriormente, n�s estimamos s1, s2, 
% s1_medio e s2_medio (Segui os passos do Raul)

% No entanto, creio que isto n�o seja necess�rio, pois n�s temos os dados
% para s1 e s2 (observ�veis), e portanto n�o necessidadede estim�-los.

% Procedendo dessa maneira, a resolu��o do item fica:

% Computando os par�metros estimados no item anterior:
alpha1_hat = coef_2SLS(1)
alpha2_hat = coef_2SLS(2)
gamma_hat  = coef_2SLS(3)
lambda_hat = coef_2SLS(4)
beta_hat   = - lambda_hat

% Calculando agora o custo marginal m�dio estimado, a partir da express�o obtida atrav�s da CPO da firma:
cmg1 = mean(data.p1) + 1/(beta_hat * (mean(data.s1) - 1))
cmg2 = mean(data.p2) + 1/(beta_hat * (mean(data.s2) - 1))

cmg = [cmg1 cmg2]

% Custo marginal m�dio da firma 1 (ceu)  = 70.3107
% Custo marginal m�dio da firma 2 (rede) = 70.1736


%% Item 8

% Primeiro, criamos a Fun��o Market-Share em outro script.

% Na falta de valores razoaveis, como o item 7 mandava usar zero
% para as caracteristicas nao-observaveis, seguimos com a recomendacao:
greek1 = 0
greek2 = 0

% Criando as vari�veis x1 e x2 m�dias (m�dia dos canais), mesma suposi��o
% de mercado fict�cio do item 7:
x1 = mean(data.x1)
x2 = mean(data.x2)


% Iremos resolver numericamente o sistema de CPOs montado no papel, por meio de
% uma itera��o usando um while loop.

% Definindo os par�metros de itera��o:

error = 1           % Valor inicial para 'error', sen�o o loop n�o come�a
tol = 10^(-5)       % Toler�ncia para o crit�rio de converg�ncia
iter = 0            % Iniciador para a contagem
maxiter = 2000      % N�mero m�ximo de itera��es. Importante para evitar que o c�digo rode para sempre caso haja algum bug
p_old = cmg'        % Vetor de pre�os iniciais. Creio que n�o importa de onde comece


% Operando a itera��o via while loop:

% Fun��o Market-Share. Ela est� programada e salva em outro arquivo. Deixarei em formato de coment�rio:

% function [output1, output2] = market_share_function(p1, p2, x1, x2, alpha1, alpha2, gamma, beta, greek1, greek2)
% 
% v1 = exp(alpha1 + gamma * x1 - beta * p1 + greek1);
% v2 = exp(alpha2 + gamma * x2 - beta * p2 + greek2);
% 
% mkt_share1 = v1/(1 + v1 + v2);
% mkt_share2 = v2/(1 + v1 + v2);
% 
% output1 = mkt_share1;
% output2 = mkt_share2;
% 
% 
% end

% Definindo o diret�rio onde est� salva a fun��o market_share_function.m:
cd('C:\Users\joseg_000.PC-JE\Google Drive\FGV-EPGE\Microeconomia\Micro 3\Listas de Exerc�cios\Minhas Resolu��es\Lista 8 - Micro 3')

% Analisando os pre�os de equil�brio no caso em que as firmas se fundem,
% mas n�o ocorre a redu��o no custo marginal que elas alegam:

while error > tol & iter < maxiter
    
    iter = iter + 1
    
    [s(1) s(2)] = market_share_function(p_old(1), p_old(2), x1, x2, alpha1_hat, alpha2_hat, gamma_hat, beta_hat, greek1, greek2)

    M = - (1/beta_hat) * inv( [ s(1)*(s(1) - 1)   s(1)*s(2) ; s(1)*s(2)    s(2)*(s(2) - 1) ] )
    
    p_new = M * s' + cmg'  % Vem da CPO da firma ap�s a fus�o - � a melhor resposta
                                      
    error = max(abs(p_new - p_old))  % Verificando se j� chegamos no equil�brio
    
    p_old = p_new                   % Caso n�o tenha chegado no equil�brio (converg�ncia para o ponto fixo),
                                    % reiniciamos o processo.
                                    
end
    
preco_fusao = [p_new(1) p_new(2)]

% Os pre�os de equil�brio neste caso s�o p1 = 159.5423 e p2 = 159.4052


% Analisando os pre�os de equil�brio no caso em que as firmas se fundem,
% e de fato ocorre a redu��o no custo marginal de 15% que elas alegam:

% Redefinindo os par�metros de itera��o:
error = 1           % Valor inicial para 'error', sen�o o loop n�o come�a
tol = 10^(-5)       % Toler�ncia para o crit�rio de converg�ncia
iter = 0            % Iniciador para a contagem
maxiter = 2000      % N�mero m�ximo de itera��es. Importante para evitar que o c�digo rode para sempre caso haja algum bug
p_old = cmg'        % Vetor de pre�os iniciais, note

while error > tol & iter < maxiter
    
    iter = iter + 1
    
    [s(1) s(2)] = market_share_function(p_old(1), p_old(2), x1, x2, alpha1_hat, alpha2_hat, gamma_hat, beta_hat, greek1, greek2)

    M = - (1/beta_hat) * inv( [ s(1)*(s(1) - 1)   s(1)*s(2) ; s(1)*s(2)    s(2)*(s(2) - 1) ] )
    
    p_new = M * s' + cmg'*(1 - 0.15)  % Vem da CPO da firma ap�s a fus�o - � a melhor resposta
                                      % Note que o custo marginal est� reduzido em 15 neste caso
    
    error = max(abs(p_new - p_old))   % Verificando se j� chegamos no equil�brio
    
    p_old = p_new                     % Caso n�o tenha chegado no equil�brio (converg�ncia para o ponto fixo),
                                      % reiniciamos o processo.
end
    
preco_fusao_reduzido = [p_new(1) p_new(2)]

% Os pre�os de equil�brio neste caso s�o p1 = 152.1244 e p2 = 152.0078


% J� os pre�os quando as duas firmas operam em oligop�lio � dado pela m�dia
% dos pre�os nos mercados, que temos na base de dados:
preco_oligopolio = [mean(data.p1) mean(data.p2)]


% Comparando os pre�os nos 3 casos:
comparacao = [preco_fusao; preco_fusao_reduzido; preco_oligopolio]


%               PRE�OS DE EQUIL�BRIO
%                                      P1         P2
% Pre�o fus�o                       159.5423   159.4052
% Pre�o fus�o com custo reduzido    152.1244   152.0078
% Pre�o de duop�lio                 149.1320   144.3340

% Conclu�mos portanto que mesmo que a fus�o das firmas traga o alegado
% ganho de efici�ncia e redu��o de custos, o aumento de poder de mercado
% ser� tal que o pre�o final ao consumidor ser� maior do que no caso em que
% as duas firmas competem em oligop�lio.

