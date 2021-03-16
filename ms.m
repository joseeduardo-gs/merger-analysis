function [s1hat,s2hat]=ms(p_hat,teta2,mx1,mx2)
s1hat=(exp(teta2(1)+teta2(3)*mx1+teta2(4)*p_hat(1)))/(1+exp(teta2(1)+teta2(3)*mx1+teta2(4)*p_hat(1)) + exp(teta2(2)+teta2(3)*mx2 + teta2(4)*p_hat(2)));
s2hat=(exp(teta2(2)+teta2(3)*mx2+teta2(4)*p_hat(2)))/(1+exp(teta2(1)+teta2(3)*mx1+teta2(4)*p_hat(1)) + exp(teta2(2)+teta2(3)*mx2 + teta2(4)*p_hat(2)));

end

