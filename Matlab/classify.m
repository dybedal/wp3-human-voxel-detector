close all;

% Subscribe to point cloud topic
pc_sub = rossubscriber('/pointcloud_merger/pointcloud_out')
pos_pub = rospublisher('visualization_marker','visualization_msgs/Marker')

xdet = 0.0;
ydet = 0.0;
zdet = 0.0;

minx = 0.5;
miny = 0.5;
maxx = 9.5;
maxy = 9.5;

minz = 0.1;
maxz = 3.5;

dc = 0.04;

rounds = 1000;

net = baselineCNN;

inputSize = net.Layers(1).InputSize;
     
seq = 0;

for r = 1:rounds
	
	% Get a point cloud
	pointcloud = receive(pc_sub,60);

	% Save to Matlab Point cloud format
	pc = pointCloud(readXYZ(pointcloud),'intensity',readField(pointcloud,'intensity'));

	% Segment the point cloud
	minDistance = 0.20;
	[labels,numClusters] = pcsegdist(pc,minDistance);

	% Plot the different segments
%	figure;
%	pcshow(pc.Location,labels);
%	colormap(hsv(numClusters));

	% Find clusters with correct size for humans:
	j = 0;

    id = 0;
	
	idx = zeros(1);
	lbl = zeros(1);
	for i = 1:numClusters
		
		[row,col] = find(labels==i);
		
		cloud = select(pc,row);
		
		xd = cloud.XLimits(2) - cloud.XLimits(1);
		yd = cloud.YLimits(2) - cloud.YLimits(1);
		zd = cloud.ZLimits(2) - cloud.ZLimits(1);

        xpos = cloud.XLimits(2) - xd/2;
        ypos = cloud.YLimits(2) - yd/2;
        zpos = cloud.ZLimits(2) - zd/2;
        
		
		if (0.2 < xd) && (xd < 2.0) && (0.2 < yd) && (yd < 2.0) && (0.5 < zd) && (zd < 2.5) && (cloud.ZLimits(1) < 0.5)

			
			
 			if(j == 0)
 				idx = row;
				lbl = ones(size(row)).*j;
 			else
 				idx = [idx; row];
				lbl = [lbl; ones(size(row)).*j];
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


            imX = mat2gray(flip(picXz));
			imwrite(imX, 'tmp/tmpX.jpg');
            
            imY = mat2gray(flip(picYz));
 			imwrite(imY, 'tmp/tmpY.jpg');
           
            
            imds = imageDatastore(fullfile('tmp'), 'FileExtensions','.jpg');


            augimds = augmentedImageDatastore(inputSize(1:2),imds,...
                'ColorPreprocessing', 'gray2rgb');

            %tbf = rostime('now');
            predictedLabel = classify(baselineCNN,augimds);
            
			%tn = rostime('now');
			%tn - tbf
			
			if all(predictedLabel == 'Human')
                
				xdet = xpos
				ydet = ypos
				zdet = zpos
                %figure;
      			%image(imX,'CDataMapping','scaled');
      			%colorbar;
                
                %figure;
      			%image(imY,'CDataMapping','scaled');
      			%colorbar;
				
				%figure;
				%pcshow(cloud);
                
                id = id+1;
                
				msg = rosmessage(pos_pub);
				
				msg.Header.FrameId = 'world';
                msg.Header.Stamp = rostime(0);
				
                msg.Header.Seq = seq;
                seq = seq+1;
               
                msg.Ns = 'detections';
                msg.Id = id;
                msg.Type = 1;
                msg.Action = 0;
       
				msg.Pose.Position.X = xpos;
                msg.Pose.Position.Y = ypos;
                msg.Pose.Position.Z = zpos;
				
                msg.Pose.Orientation.X = 0.0;
                msg.Pose.Orientation.Y = 0.0;
                msg.Pose.Orientation.Z = 0.0;
                msg.Pose.Orientation.W = 1.0;
				
                msg.Scale.X = xd;
                msg.Scale.Y = yd;
                msg.Scale.Z = zd;
                
                msg.Color.R = 0.0;
                msg.Color.G = 1.0;
                msg.Color.B = 0.0;
                msg.Color.A = 0.5;
                
                %msg.Lifetime = rosduration(0.75);
				msg.Lifetime = rosduration(0);
                
                send(pos_pub,msg)
								
			end		
			
		end

	end
	
	%cld = select(pc,idx)
	%figure;
	%pcshow(cld.Location,lbl);
	%colormap(hsv(numClusters));
	

end

rosshutdown;
