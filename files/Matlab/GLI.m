function Tij = GLI(a,b,c,d)

rac = c-a;
rad = d-a;
rbc = c-b;
rbd = d-b;

na = cross(rac,rad)/norm(cross(rac,rad));
nb = cross(rad,rbd)/norm(cross(rad,rbd));
nc = cross(rbd,rbc)/norm(cross(rbd,rbc));
nd = cross(rbc,rac)/norm(cross(rbc,rac));

Tij = asin(dot(na,nb)) + asin(dot(nb,nc)) + asin(dot(nc,nd)) + asin(dot(nd,na));
Tij = Tij/(4*3.14);