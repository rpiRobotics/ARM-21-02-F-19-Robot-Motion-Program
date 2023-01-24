A = readmatrix('original/Curve_backproj_in_base_frame.csv');
curve=A(1:end,1:3);
curve_normal=A(1:end,4:6);
distance=[0];
for i=2:length(curve)
    distance=[distance;norm(curve(i,:)-curve(i-1,:))];
end
%%%equally distributed index based on distance
x=[0];
for i=2:length(curve)
    x=[x;x(end)+distance(i)/sum(distance)];
end
%%%define interp index every 0.1mm
xq=linspace(0,1,sum(distance)/0.1);
curve_interp = interp1(x,curve,xq,'spline');
curve_normal_interp=[];

%%%interp curve normal
for i=1:length(curve_interp)
   %%%find closest 2 points on original curve
   curve_temp=curve-(curve_interp(i,:)'*ones(1,length(curve)))';
   [out,idx]=sort(vecnorm(curve_temp')');
   axis=cross(curve_normal(idx(1),:),curve_normal(idx(2),:));
   angle_main= atan2(norm(cross(curve_normal(idx(1),:),curve_normal(idx(2),:))), dot(curve_normal(idx(1),:),curve_normal(idx(2),:)));
   angle_interp=angle_main*norm(curve_interp(i,:)-curve(idx(1),:))/distance(ceil(idx(1)/2+idx(2)/2));

   axang = [axis angle_interp];
   rotm = axang2rotm(axang);
   curve_normal_interp=[curve_normal_interp ; (rotm*curve_normal(idx(1),:)')'];
   

   
   
end
% plot3(curve_interp(:,1),curve_interp(:,2),curve_interp(:,3))


writematrix([curve_interp curve_normal_interp],'from_interp/Curve_backproj_in_base_frame.csv')