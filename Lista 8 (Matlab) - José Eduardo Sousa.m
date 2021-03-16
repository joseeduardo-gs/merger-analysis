% Lista 8 - Produtos Diferenciados - Microeconomia III (2018)
% Aluno: José Eduardo Sousa

clear clear all clc

% Definindo o diretório de trabalho:
cd('C:\Users\joseg_000.PC-JE\Google Drive\FGV-EPGE\Microeconomia\Micro 3\Listas de Exercícios\Minhas Resoluções\Lista 8 - Micro 3\data')

% Importando os dados:
data = readtable('data.txt')


%% Item 3

% Vamos agora organizar a base de dados para que possamos rodar a regressão
% OLS do modelo descrito no item 2.

% Para isso, vamos empilhar o modelo e criar as variáveis necessárias:

% Empilhando as variáveis market-share, preço e canais:
market_share = [data.s1 ; data.s2]
preco = [data.p1 ; data.p2]
canais = [data.x1 ; data.x2]
z = [data.z1; data.z2]

% Market-Share da outside option, e empilhando:
S0 = [ (1 - data.s1 - data.s2) ; (1 - data.s1 - data.s2) ]

% Criando a variável dependente:
diff_log_mkt_share = log(market_share) - log(S0)
% ou
diff_log_mkt_share = log(market_share ./ S0)

% Criando uma variáveis dummy para diferenciar o intercepto entre as firmas 1 e 2:

% Dummy de da firma Céu:
% Se j = 1, a dummy ceu assume valor ceu = 1
% Se j = 2, a dummy ceu assume valor ceu = 0, pois se trata da firma "rede"
ceu = [ ones(500, 1) ; zeros(500, 1)]

% Dummy de da firma Rede:
% Se j = 1, a dummy rede assume valor rede = 0, , pois se trata da firma "Céu"
% Se j = 2, a dummy ceu assume valor rede = 1
rede = [ zeros(500, 1) ; ones(500, 1)]

% Adicionando todas as variáveis necessárias para a regressão em uma única base de dados empilhada:
dados = [diff_log_mkt_share ones(1000, 1) ceu rede canais preco]

Y = diff_log_mkt_share                      % Vetor da variável dependente
X = [ones(1000, 1) ceu canais preco]        % Matriz de Regressores

% Regressão OLS de diff_log_mkt_share em ceu, canais e preços, com constante:
coeficientes_ols = regress(Y, X)

% A regressão OLS nos deu os seguintes coeficientes estimados:

% (Intercepto) -2.0664      --->    alpha1 = Intercepto + ceu = -1.7545
% ceu           0.3119      --->    alpha2 = Intercepto = -2.0664
% canais        0.0040      --->    gamma  = canais = 0.0040
% preco         0.0007      --->    - beta = lambda = preco = 0.0007
%                           --->      beta = - 0.0007



% A regressão também poderia ter sido feita com uma dummy para cada firma, e sem constante:

Y = diff_log_mkt_share                         % Vetor da variável dependente
X_alternativo = [ceu rede canais preco]        % Matriz de Regressores

% Regressão OLS de diff_log_mkt_share em ceu, canais e preços, com constante:
coeficientes_ols_alternativo = regress(Y, X_alternativo)

% A regressão OLS nos deu os seguintes coeficientes estimados:

%   alpha1 = -1.7545   (Intercepto Firma 1 - ceu)
%   alpha2 = -2.0664   (Intercepto Firma 2 - rede)
%   gamma  = 0.0040    (Canais)
%   - beta = lambda = preco = 0.0007 ------->>  beta = - 0.0007 (Preço)


%% Item 3 (De outra Maneira - Seguindo a dica do enunciado):

% A variável dependente é construída da mesma maneira

% Vetor de Regressores:

X_dica = [ ones(500, 1) zeros(500, 1) data.x1 data.p1 ; zeros(500, 1) ones(500, 1) data.x2 data.p2 ]

% Note que a diferença entre esta matriz de regressores e a feita anteriormente é a seguinte:
%   - Em X há um intercepto (grupo base "Rede") e uma dummy para a firma "ceu"
%   - Em X_dica não há intercepto, e há uma dummy para cada firma, ceu (coluna 1) e rede (coluna 2).
%   - As colunas 3 e 4 que representam as variaveis x (canal) e p (preço) são exatamente as mesmas.


% Regressão:

coeficientes_ols_2 = regress(Y, X_dica)

% A regressão OLS nos deu os seguintes coeficientes estimados:

%   alpha1 = -1.7545   (Intercepto Firma 1 - ceu)
%   alpha2 = -2.0664   (Intercepto Firma 2 - rede)
%   gamma  = 0.0040    (Canais)
%   - beta = lambda = preco = 0.0007 ------->>  beta = - 0.0007 (Preço)


%% Item 6:

% Para corrigir o problema de endogeneidade do modelo anterior, iremos usar 
% a variável z (estimativa do custo marginal) como instrumento, e faremos a
% estimação pelo método de Mínimos Quadrados em 2 Estágios (MQ2E):

% Variável endógena: preco
P = preco

% Intrumento:
z = [data.z1 ; data.z2]

% Matriz de Regressores exógenos e instrumentos:
Z = [ ones(500, 1) zeros(500, 1) data.x1 data.z1 ; zeros(500, 1) ones(500, 1) data.x2 data.z2 ]

% Outra forma de montar a matriz Z, usando as variáveis empilhadas criadas antes:
Z = [ceu, rede, canais, z]

% Primeiro Estágio: Regressão da variável endógena nas variáveis exógenas e nos instrumentos (P em Z):
coef_estagio = regress(P, Z)

% Computando o valor ajustado de P (projeção de P em Z):
P_hat = Z * coef_estagio


% Segundo estágio: Reestimamos a equação original, mas substituindo a variável endógena P por P_hat:

X_vi = [ceu rede canais P_hat]       % Nova matriz de regressores.

coef_2SLS = regress(Y, X_vi)


% A regressão MQ2E nos deu os seguintes coeficientes estimados:

%   alpha1 = 0.6043   (Intercepto Firma 1 - ceu)
%   alpha2 = 0.2140   (Intercepto Firma 2 - rede)
%   gamma  = 0.0048    (Canais)
%   - beta = lambda = preco = -0.0157 ------->>  beta = 0.0157 (Preço)



%% Item 7: (Acho que está maneira está incorreta - ver próximo tópico)
% 
% % Computando os parâmetros estimados no item anterior:
% alpha1_hat = coef_2SLS(1)
% alpha2_hat = coef_2SLS(2)
% gamma_hat  = coef_2SLS(3)
% lambda_hat = coef_2SLS(4)
% beta_hat   = - lambda_hat
% 
% % Calculando o Market-Share médio, com choque nulo:
% v1 = exp( alpha1_hat + gamma_hat * mean(data.x1) - beta_hat * mean(data.p1) )
% v2 = exp( alpha2_hat + gamma_hat * mean(data.x2) - beta_hat * mean(data.p2) )
% 
% s1_medio = v1/(1 + v1 + v2)
% s2_medio = v2/(1 + v1 + v2)
% 
% % Calculando agora o custo marginal médio estimado:
% cmg1 = mean(data.p1) + 1/(beta_hat * (s1_medio - 1))
% cmg2 = mean(data.p2) + 1/(beta_hat * (s2_medio - 1))
% 
% % Custo marginal médio da firma 1 (ceu)  = 70.5032
% % Custo marginal médio da firma 2 (rede) = 70.3510
% 
% 
% 
% % Outra maneira de calcular o market-share médio, calculando o market-share
% % estimado para cada t, e depois tirando a média:
% v1_t = exp( alpha1_hat + gamma_hat * data.x1 - beta_hat * data.p1 )
% v2_t = exp( alpha2_hat + gamma_hat * data.x2 - beta_hat * data.p2 )
% 
% s1_t = v1_t ./(1 + v1_t + v2_t)
% s2_t = v2_t ./(1 + v1_t + v2_t)
% 
% s1_MEDIO = mean(s1_t)
% s2_MEDIO = mean(s2_t)
% 
% % Calculando agora o custo marginal médio estimado:
% cmg1_ = mean(data.p1) + 1/(beta_hat * (s1_MEDIO - 1))
% cmg2_ = mean(data.p2) + 1/(beta_hat * (s2_MEDIO - 1))
% 
% cmg = [cmg1 cmg2]
% 
% % Custo marginal médio da firma 1 (ceu)  = 70.4505
% % Custo marginal médio da firma 2 (rede) = 70.3079


%% Item 7 (Outra Maneira - Creio que mais correta)

% Do jeito que foi feito o item 7 anteriormente, nós estimamos s1, s2, 
% s1_medio e s2_medio (Segui os passos do Raul)

% No entanto, creio que isto não seja necessário, pois nós temos os dados
% para s1 e s2 (observáveis), e portanto não necessidadede estimá-los.

% Procedendo dessa maneira, a resolução do item fica:

% Computando os parâmetros estimados no item anterior:
alpha1_hat = coef_2SLS(1)
alpha2_hat = coef_2SLS(2)
gamma_hat  = coef_2SLS(3)
lambda_hat = coef_2SLS(4)
beta_hat   = - lambda_hat

% Calculando agora o custo marginal médio estimado, a partir da expressão obtida através da CPO da firma:
cmg1 = mean(data.p1) + 1/(beta_hat * (mean(data.s1) - 1))
cmg2 = mean(data.p2) + 1/(beta_hat * (mean(data.s2) - 1))

cmg = [cmg1 cmg2]

% Custo marginal médio da firma 1 (ceu)  = 70.3107
% Custo marginal médio da firma 2 (rede) = 70.1736


%% Item 8

% Primeiro, criamos a Função Market-Share em outro script.

% Na falta de valores razoaveis, como o item 7 mandava usar zero
% para as caracteristicas nao-observaveis, seguimos com a recomendacao:
greek1 = 0
greek2 = 0

% Criando as variáveis x1 e x2 médias (média dos canais), mesma suposição
% de mercado fictício do item 7:
x1 = mean(data.x1)
x2 = mean(data.x2)


% Iremos resolver numericamente o sistema de CPOs montado no papel, por meio de
% uma iteração usando um while loop.

% Definindo os parâmetros de iteração:

error = 1           % Valor inicial para 'error', senão o loop não começa
tol = 10^(-5)       % Tolerância para o critério de convergência
iter = 0            % Iniciador para a contagem
maxiter = 2000      % Número máximo de iterações. Importante para evitar que o código rode para sempre caso haja algum bug
p_old = cmg'        % Vetor de preços iniciais. Creio que não importa de onde comece


% Operando a iteração via while loop:

% Função Market-Share. Ela está programada e salva em outro arquivo. Deixarei em formato de comentário:

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

% Definindo o diretório onde está salva a função market_share_function.m:
cd('C:\Users\joseg_000.PC-JE\Google Drive\FGV-EPGE\Microeconomia\Micro 3\Listas de Exercícios\Minhas Resoluções\Lista 8 - Micro 3')

% Analisando os preços de equilíbrio no caso em que as firmas se fundem,
% mas não ocorre a redução no custo marginal que elas alegam:

while error > tol & iter < maxiter
    
    iter = iter + 1
    
    [s(1) s(2)] = market_share_function(p_old(1), p_old(2), x1, x2, alpha1_hat, alpha2_hat, gamma_hat, beta_hat, greek1, greek2)

    M = - (1/beta_hat) * inv( [ s(1)*(s(1) - 1)   s(1)*s(2) ; s(1)*s(2)    s(2)*(s(2) - 1) ] )
    
    p_new = M * s' + cmg'  % Vem da CPO da firma após a fusão - é a melhor resposta
                                      
    error = max(abs(p_new - p_old))  % Verificando se já chegamos no equilíbrio
    
    p_old = p_new                   % Caso não tenha chegado no equilíbrio (convergência para o ponto fixo),
                                    % reiniciamos o processo.
                                    
end
    
preco_fusao = [p_new(1) p_new(2)]

% Os preços de equilíbrio neste caso são p1 = 159.5423 e p2 = 159.4052


% Analisando os preços de equilíbrio no caso em que as firmas se fundem,
% e de fato ocorre a redução no custo marginal de 15% que elas alegam:

% Redefinindo os parâmetros de iteração:
error = 1           % Valor inicial para 'error', senão o loop não começa
tol = 10^(-5)       % Tolerância para o critério de convergência
iter = 0            % Iniciador para a contagem
maxiter = 2000      % Número máximo de iterações. Importante para evitar que o código rode para sempre caso haja algum bug
p_old = cmg'        % Vetor de preços iniciais, note

while error > tol & iter < maxiter
    
    iter = iter + 1
    
    [s(1) s(2)] = market_share_function(p_old(1), p_old(2), x1, x2, alpha1_hat, alpha2_hat, gamma_hat, beta_hat, greek1, greek2)

    M = - (1/beta_hat) * inv( [ s(1)*(s(1) - 1)   s(1)*s(2) ; s(1)*s(2)    s(2)*(s(2) - 1) ] )
    
    p_new = M * s' + cmg'*(1 - 0.15)  % Vem da CPO da firma após a fusão - é a melhor resposta
                                      % Note que o custo marginal está reduzido em 15 neste caso
    
    error = max(abs(p_new - p_old))   % Verificando se já chegamos no equilíbrio
    
    p_old = p_new                     % Caso não tenha chegado no equilíbrio (convergência para o ponto fixo),
                                      % reiniciamos o processo.
end
    
preco_fusao_reduzido = [p_new(1) p_new(2)]

% Os preços de equilíbrio neste caso são p1 = 152.1244 e p2 = 152.0078


% Já os preços quando as duas firmas operam em oligopólio é dado pela média
% dos preços nos mercados, que temos na base de dados:
preco_oligopolio = [mean(data.p1) mean(data.p2)]


% Comparando os preços nos 3 casos:
comparacao = [preco_fusao; preco_fusao_reduzido; preco_oligopolio]


%               PREÇOS DE EQUILÍBRIO
%                                      P1         P2
% Preço fusão                       159.5423   159.4052
% Preço fusão com custo reduzido    152.1244   152.0078
% Preço de duopólio                 149.1320   144.3340

% Concluímos portanto que mesmo que a fusão das firmas traga o alegado
% ganho de eficiência e redução de custos, o aumento de poder de mercado
% será tal que o preço final ao consumidor será maior do que no caso em que
% as duas firmas competem em oligopólio.

