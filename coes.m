function [ h, i, RAAN, ecc, omeg, theta ] = coes( r_0, v_0, mu )
% R,V to COES

vr_0=dot(r_0,v_0)/norm(r_0);

%find h
H=cross(r_0,v_0);
h=norm(H);

%find inclination
i=acosd(H(3)/norm(H)); %degrees

%find right ascension of the ascending node
N=cross([0 0 1], H);
if norm(N)~=0
    RAAN=acosd(N(1)/norm(N));
    if N(2)<0
        RAAN=360-RAAN;
    end
else 
    RAAN=0;
end

%find eccentricity
ecc_vect=1/mu*(cross(v_0,H)-mu*r_0/norm(r_0));
ecc=norm(ecc_vect);

%find argument of periapse
if norm(N)~=0
    if ecc>eps
        omeg=acosd(dot(N,ecc_vect)/(norm(N)*ecc));
        if ecc_vect(3)<0
            omeg=360-omeg;
        end
    else
        omeg=0;
    end
else
    omeg=0;
end

%find true anamoly
if ecc > eps
    theta=acosd(dot(ecc_vect,r_0)/(ecc*norm(r_0)));
    if vr_0 < 0
        theta=360-theta;
    end
else
    circ=cross(N,r_0);
    if circ(3)>0
        theta=acosd(dot(N,r_0)/(norm(N)*norm(r_0)));
    else
        theta=360-acosd(dot(N,r_0)/(norm(N)*norm(r_0)));
    end 
end
 end

