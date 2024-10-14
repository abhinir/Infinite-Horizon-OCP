r10 = [-6296.30;817.09;3591.96];
v10 = [-3.05;-6.00;-2.65];
r20 = [-2192.51;-6164.08;-1033.77];
v20 = [3.12;-1.68;-7.24];
mu = 398600.4418;
N = 3000;
dt = 2;

r1 = zeros(3,N+1);
v1 = zeros(3,N+1);
r1(:,1) = r10;
v1(:,1) = v10;

r2 = zeros(3,N+1);
v2 = zeros(3,N+1);
r2(:,1) = r20;
v2(:,1) = v20;

p = 1;
for i = 0:dt:N
    r1(:,p+1) = r1(:,p) + v1(:,p)*dt;
    R1 = norm(r1(:,p));
    v1(:,p+1) = v1(:,p) - (mu/(R1^3))*r1(:,p)*dt;
    r2(:,p+1) = r2(:,p) + v2(:,p)*dt;
    R2 = norm(r2(:,p));
    v2(:,p+1) = v2(:,p) - (mu/(R2^3))*r2(:,p)*dt;
    p = p + 1;
end

p = 1; 
% for i = 0:dt:3000
%     plot3(r1(1,p),r1(2,p),r1(3,p),'k')
%     hold on
%     plot3(r2(1,p),r2(2,p),r2(3,p),'b')
%     hold on
%     pause(0.001)
%     p = p + 1;
% end
plot3(r1(1,:),r1(2,:),r1(3,:))
hold on
plot3(r1(1,:)-X(1,:),r1(2,:)-X(2,:),r1(3,:)-X(3,:))
% 
% 
% for i=1:10
%     plot(linspace(0,1),linspace(0,1))
%     hold on;pause(0.1);
% end