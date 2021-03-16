function [output1, output2] = market_share_function(p1, p2, x1, x2, alpha1, alpha2, gamma, beta, greek1, greek2)

v1 = exp(alpha1 + gamma * x1 - beta * p1 + greek1);
v2 = exp(alpha2 + gamma * x2 - beta * p2 + greek2);

mkt_share1 = v1/(1 + v1 + v2);
mkt_share2 = v2/(1 + v1 + v2);

output1 = mkt_share1;
output2 = mkt_share2;


end

