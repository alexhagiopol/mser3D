function levMarErrors(levMarIterations)
correctValues = csvread('/home/alex/mser/mser_3d/build/Debug/syntheticOptimizationTestResults.csv',2,0,[2,0,2,7]);
errorMatrix = csvread('/home/alex/mser/mser_3d/build/Debug/syntheticOptimizationTestResults.csv',5,0,[5,0,levMarIterations + 4,8]);
xErrors = errorMatrix(:,2);
yErrors = errorMatrix(:,3);
zErrors = errorMatrix(:,4);
rollErrors = errorMatrix(:,5);
pitchErrors = errorMatrix(:,6);
yawErrors = errorMatrix(:,7);
majAxisErrors = errorMatrix(:,8);
minAxisErrors = errorMatrix(:,9);

i = linspace(1,levMarIterations,levMarIterations);
figure;
%X
subplot(2,4,1);
plot(i,xErrors,'r.');
xlabel('# Lev Mar Iterations');
ylabel('X Error');
title('X Error');
%Y
subplot(2,4,2);
plot(i,yErrors,'g.');
xlabel('# Lev Mar Iterations');
ylabel('Y Error');
title('Y Error');
%Z
subplot(2,4,3);
plot(i,zErrors,'b.');
xlabel('# Lev Mar Iterations');
ylabel('Z Error');
title('Z Error');
%Roll
subplot(2,4,4);
plot(i,rollErrors,'c.');
xlabel('# Lev Mar Iterations');
ylabel('Roll Error');
title('Roll Error');
%Pitch
subplot(2,4,5);
plot(i,pitchErrors,'m.');
xlabel('# Lev Mar Iterations');
ylabel('Pitch Error');
title('Pitch Error');
%Yaw
subplot(2,4,6);
plot(i,yawErrors,'y.');
xlabel('# Lev Mar Iterations');
ylabel('Yaw Error');
title('Yaw Error');
%Maj Axis
subplot(2,4,7);
plot(i,majAxisErrors,'k.');
xlabel('# Lev Mar Iterations');
ylabel('Maj Axis Error');
title('Maj Axis Error');
%Min Axis
subplot(2,4,8);
plot(i,minAxisErrors,'k*');
xlabel('# Lev Mar Iterations');
ylabel('Min Axis Error');
title('Min Axis Error');
end

