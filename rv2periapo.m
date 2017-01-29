function [ r_a, r_p ] = rv2periapo( r, v, mu )

r_a = ones(length(r(:,1)),1);
r_p = ones(length(r(:,1)),1);

for i=1:length(r_a)
    [ h(i), inc(i), RAAN(i), ecc(i), omeg(i), theta(i) ] = coes( r(i,:), v(i,:), mu );
    r_p(i) = h(i)^2/mu*1/(1+ecc(i));
    r_a(i) = r_p(i)/((1-ecc(i))/(1+ecc(i)));
end


end

