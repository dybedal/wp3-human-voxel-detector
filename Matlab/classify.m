close all;

% Initialize ROS
%rosinit;

% Subscribe to point cloud topic
pc_sub = rossubscriber('/pointcloud_merger/pointcloud_out');

minx = 0.5;
miny = 0.5;
maxx = 9.5;
maxy = 9.5;

minz = 0.1;
maxz = 3.5;

dc = 0.04;

rounds = 100;

t = datestr(now());
mkdir(t);
            
for r = 1:rounds
	
	% Get a point cloud
	pointcloud = receive(pc_sub,100);

	% Save to Matlab Point cloud format
	pc = pointCloud(readXYZ(pointcloud),'intensity',readField(pointcloud,'intensity'));

	% Segment the point cloud
	minDistance = 0.2;
	[labels,numClusters] = pcsegdist(pc,minDistance);

	% Plot the different segments
	% figure;
	% pcshow(pc.Location,labels);
	% colormap(hsv(numClusters));

	% Find clusters with correct size for humans:
	j = 0;


	
	for i = 1:numClusters

		[row,col] = find(labels==i);

		cloud = pointCloud(pc.Location(row,:),'intensity',pc.Intensity(row,:));
		xd = cloud.XLimits(2) - cloud.XLimits(1);
		yd = cloud.YLimits(2) - cloud.YLimits(1);
		zd = cloud.ZLimits(2) - cloud.ZLimits(1);


		if (0.2 < xd) && (xd < 2.0) && (0.2 < yd) && (yd < 2.0) && (0.5 < zd) && (zd < 2.5) && (cloud.ZLimits(1) < 0.5)

			if(j == 0)
				idx = row;
			else
				idx = [idx; row];
			end


			j = j+1;


			% Make pics
			zxdim = [(ceil( (zd)/dc )+1) (ceil( (xd)/dc )+1)];
			zydim = [(ceil( (zd)/dc )+1) (ceil( (yd)/dc )+1)];

			picXz = zeros(zxdim);
			picYz = zeros(zydim);

			for k = 1:cloud.Count
				x = round( (cloud.Location(k,1) - cloud.XLimits(1) ) / dc) + 1;
				y = round( (cloud.Location(k,2) - cloud.YLimits(1) ) / dc) + 1;
				z = round( (cloud.Location(k,3) - cloud.ZLimits(1) ) / dc) + 1;
				picXz(z,x) = picXz(z,x) + cloud.Intensity(k);
				picYz(z,y) = picYz(z,y) + cloud.Intensity(k);
			end

% 			figure;
% 			image(cloud.XLimits,cloud.ZLimits,picXz,'CDataMapping','scaled');
% 			set(gca,'YDir','normal');
% 			colorbar;
			
			imXz = mat2gray(flip(picXz));

% 			figure;
% 			image(cloud.YLimits,cloud.ZLimits,picYz,'CDataMapping','scaled');
% 			set(gca,'YDir','normal');
% 			colorbar;

			imYz = mat2gray(flip(picYz));

			predictionX = classify(baselineCNN,imXz)
			predictionY = classify(baselineCNN,imYz)
			
			
		end


	end
% 	if numClusters > 0
% 		pcseg = pointCloud(pc.Location(idx,:),'intensity',pc.Intensity(idx,:));
% 		labelseg = labels(idx,:);
% 
% 		figure;
% 		pcshow(pcseg.Location,labelseg);
% 		colormap(hsv(j));
% 	end

end

%rosshutdown;
