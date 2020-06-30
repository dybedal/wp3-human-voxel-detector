close all;

% Initialize ROS
%rosinit;

% Subscribe to point cloud topic

net = baselineCNN;

inputSize = net.Layers(1).InputSize
            
imds = imageDatastore(fullfile('classes2'),...
'IncludeSubfolders',true,'FileExtensions','.jpg','LabelSource','foldernames');

%imds = shuffle(imds);

labelCount = countEachLabel(imds);
numClasses = height(labelCount);

augimdsTest = augmentedImageDatastore(inputSize(1:2),imds,...
    'ColorPreprocessing', 'gray2rgb');

predictedLabels = classify(baselineCNN,augimdsTest);
valLabels = imds.Labels;
baselineCNNAccuracy = sum(predictedLabels == valLabels)/numel(valLabels)


%rosshutdown;
