% data_dir='wood/'
data_dir='from_NX/'

path=fileparts(which('baseline.py'));
if count(py.sys.path,path)==0
    insert(py.sys.path,int32(0),path);
end
curve=readmatrix(data_dir+"Curve_dense.csv");

lam=double(py.baseline.calc_lam_cs(curve));
robot=py.baseline.define_robot();
disp("OPTIMIZING ON CURVE POSE")
H=py.baseline.pose_opt(robot,py.numpy.array(curve(1:end,1:3)),py.numpy.array(curve(1:end,4:6)))

tup=py.baseline.curve_frame_conversion(py.numpy.array(curve(1:end,1:3)),py.numpy.array(curve(1:end,4:6)),H);
curve_base=py.numpy.squeeze(py.numpy.array(tup(1)));
curve_normal_base=py.numpy.squeeze(py.numpy.array(tup(2)));

disp("FIND ALL POSSIBLE INV SOLUTIONS")
curve_js_all=py.baseline.find_js(robot,curve_base,curve_normal_base);
disp(['num solutions available: ',num2str(double(py.len(curve_js_all)))])

if double(py.len(curve_js_all))==0
	exit;
end

disp("FIND BEST CURVE_JS ON MIN(J_SINGULAR)")
J_min=[];
for i=1:double(py.len(curve_js_all))
	J_min=[J_min; double(py.numpy.array(py.baseline.find_j_min(robot,py.numpy.squeeze(py.numpy.array(curve_js_all(i))))))];
end

[M,I] =min(min(J_min,[],2));
curve_js=double(py.numpy.squeeze(py.numpy.array(curve_js_all(I))));

figure
plot(lam,J_min(I,:))
title('Minimum J SINGULAR')
xlabel('Lambda (mm)') 
ylabel('J (min singular value)')

figure
plot(lam,curve_js(:,1),lam,curve_js(:,2),lam,curve_js(:,2),lam,curve_js(:,3),lam,curve_js(:,4),lam,curve_js(:,5),lam,curve_js(:,6))
title('Joint Plot')
xlabel('Lambda (mm)') 
ylabel('Joints')
legend('J1','J2','J3','J4','J5','J6')