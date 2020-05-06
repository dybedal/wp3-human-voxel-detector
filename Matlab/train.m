imds = imageDatastore(fullfile('classes'),...
'IncludeSubfolders',true,'FileExtensions','.jpg','LabelSource','foldernames');

labelCount = countEachLabel(imds);
numClasses = height(labelCount);

[imdsTraining, imdsValidation] = splitEachLabel(imds, 0.7);

inputSize = [224,224,3];
augimdsTraining = augmentedImageDatastore(inputSize(1:2),imdsTraining);
augimdsValidation = augmentedImageDatastore(inputSize(1:2),imdsValidation);

layers = [
imageInputLayer([224 224 1])
convolution2dLayer(3,16,'Padding',1)
batchNormalizationLayer
reluLayer
maxPooling2dLayer(2,'Stride',2)
convolution2dLayer(3,32,'Padding',1)
batchNormalizationLayer
reluLayer
maxPooling2dLayer(2,'Stride',2)
convolution2dLayer(3,64,'Padding',1)
batchNormalizationLayer
reluLayer
fullyConnectedLayer(2)
softmaxLayer
classificationLayer];

options = trainingOptions('sgdm',...
'MaxEpochs',30, ...
'ValidationData',augimdsValidation,...
'ValidationFrequency',50,...
'InitialLearnRate', 0.0003,...
'Verbose',false,...
'Plots','training-progress');

baselineCNN = trainNetwork(augimdsTraining,layers,options);
