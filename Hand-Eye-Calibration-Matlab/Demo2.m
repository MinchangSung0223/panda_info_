%Simple script to quickly demo use of CalCamArm.m

%path to images of checkerboards
imageFolder = './examples/cal0126_2/Images';
%imageFolder = './examples/calib1211/Images';
%loading arm transformations
poselist = dir('./examples/cal0126_2/poses/*.txt');
%poselist = dir('./examples/calib1211/poses/*.txt');
armMat = zeros(4,4,length(poselist))
for i=1:length(poselist)
  loadname = './examples/cal0126_2/poses/'+string(poselist(i).name)
  %loadname = './examples/calib1211/poses/'+string(poselist(i).name)
  armMat(:,:,i) =load(loadname)
end
%checkerboard square widths in mm
squareSize = 25   ;
%run calibration
%[TBase, TEnd, cameraParams, TBaseStd, TEndStd, pixelErr] = 
[TBase, InvTB, TEnd, InvTE, cameraParams, TBaseStd, TEndStd, pixelErr] = CalCamArmEIH(imageFolder, armMat, squareSize,'maxBaseOffset',1);
%print results
fprintf('\nFinal camera to arm base transform is\n')
disp(TBase);
% 
% fprintf('Final end effector to checkerboard transform is\n')
% disp(TEnd);
% 
fprintf('Final camera matrix is\n')
disp(cameraParams.IntrinsicMatrix');
% 
% fprintf('Final camera radial distortion parameters are\n')
% disp(cameraParams.RadialDistortion);
% 
% fprintf('Final camera tangential distortion parameters are\n')
% disp(cameraParams.TangentialDistortion);
% %%%[-1 0 0 0;0 -1 0 0;0 0 1 0;0 0 0 1]
% 
% %triplot(eye(4),'frame','B','color','b','length',0.5);
% %hold on;
% %triplot(inv(TBase),'frame','K','color','r','length',0.5);
% %hold off;
% 
% 
% m=inv(TBase);
% R=m(1:3,1:3)
% Q=UnitQuaternion(R);
% S=transl(m);

% TBase =
% 
%    -0.9999    0.0101   -0.0107   -0.2116
%     0.0145    0.7825   -0.6225    0.4159
%     0.0021   -0.6225   -0.7826    0.2042
%          0         0         0    1.0000
         

% KB=inv(TBase)
% 
% KB =
% 
%    -0.9999    0.0145    0.0021   -0.2181
%     0.0101    0.7825   -0.6225   -0.1962
%    -0.0107   -0.6225   -0.7826    0.4164
%          0         0         0    1.0000
%          
         
%% Q
% 0.0033708 < 0.0065251, 0.94406, -0.3297 >

%% S    -0.2181    -0.1962     0.4164
