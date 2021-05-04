imds = imageDatastore(fullfile('classes_od'),...
'IncludeSubfolders',true,'FileExtensions','.jpg','LabelSource','foldernames');

imds = shuffle(imds);

labelCount = countEachLabel(imds);
numClasses = height(labelCount);

[imdsTraining, imdsValidation, imdsTesting] = splitEachLabel(imds, 0.6, 0.1);

augmenter = imageDataAugmenter( ...
    'RandRotation',[-90 90],...
    'RandXReflection',true,...
    'RandXShear',[-20 20],...
    'RandYShear',[-20 20]);

inputSize = [224,224,3];
augimdsTraining = augmentedImageDatastore(inputSize(1:2),imdsTraining,...
    'DataAugmentation',augmenter,'ColorPreprocessing', 'gray2rgb');
augimdsValidation = augmentedImageDatastore(inputSize(1:2),imdsValidation,...
    'DataAugmentation',augmenter, 'ColorPreprocessing', 'gray2rgb');
augimdsTesting = augmentedImageDatastore(inputSize(1:2),imdsTesting,...
    'DataAugmentation',augmenter, 'ColorPreprocessing', 'gray2rgb');

layers = [
imageInputLayer(inputSize)

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
'MaxEpochs',100, ...
'ValidationData',augimdsValidation,...
'Shuffle','every-epoch', ...
'InitialLearnRate', 0.0001,...
'LearnRateDropPeriod',10,...
'LearnRateSchedule','piecewise',...
'Verbose',false,...
'Plots','training-progress');

baselineCNN_2 = trainNetwork(augimdsTraining,layers,options);
