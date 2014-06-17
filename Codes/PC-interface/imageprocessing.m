%
%		Software: Matlab R2010b
%		Pick 'n' Place Robot Controller
%		Written by: Bharat Jain(09305029) and Dinesh Rajpoot(09305043)
%		M.Tech IIT Bombay 2009-2011
%		Last Modification: 2010-11-07
%		This program controls the working of pick 'n' place robot through
%		image processing

%  Copyright (c) 2010, ERTS Lab, IIT Bombay.                      
%  All rights reserved.

%  Redistribution and use in source and binary forms, with or without
%  modification, are permitted provided that the following conditions are met:

%  * Redistributions of source code must retain the above copyright
%    notice, this list of conditions and the following disclaimer.

%  * Redistributions in binary form must reproduce the above copyright
%   notice, this list of conditions and the following disclaimer in
%   the documentation and/or other materials provided with the
%    distribution.

%  * Neither the name of the copyright holders nor the names of
%    contributors may be used to endorse or promote products derived
%    from this software without specific prior written permission.

%  * Source code can be used for academic purpose. 
%	 For commercial use permission form the author needs to be taken.

% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE. 

%  Software released under Creative Commence cc by-nc-sa licence.
% For legal information refer to: 
%  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode


clear all;
 %clc;
s=serial('COM26'); % opening serial port, check your serial port number
s.Timeout=50;
fopen(s);


vid=videoinput('winvideo',1); % setting camera properties 
set(vid, 'FramesPerTrigger', Inf);
set(vid, 'ReturnedColorspace', 'rgb')
%config = triggerinfo(vid);
%triggerconfig(vid,config(2));

%vid.FrameGrabInterval = 5;

start(vid);

flag=0;
 while(1)
%  The different commands are;-
% 0 = Stop robot
% 1 = Go straight for 20 mm distance
% 2 = Left turn by the specified angle
% 3 = Right turn by the specified angle
% 4 = Pick the object
% 5 = Place object

%flag=0;green=0;orange=0
         %infinite loop


%trigger(vid);   
if(flag==0)          %3 conditions for flag- 0,1,2
    
    %flag
    RGB=getsnapshot(vid);
     
     
dim=size(RGB);%stores dimension of RGB matrix
%Color segmentation code
coly=zeros(dim(1),dim(2));
colb = zeros(dim(1),dim(2));
colw = zeros(dim(1),dim(2));
x3=zeros(10,1);d=zeros(10,1);
for i=1:dim(1)
    for j=1:dim(2)
        if(RGB(i,j,1) <=110 && RGB(i,j,2) < 255 && RGB(i,j,2) > 120 && RGB(i,j,3) <= 140)  %Front Tag(Green) on Firebird
            colw(i,j)=255;
          
        end
        if(RGB(i,j,1) >150 && RGB(i,j,2)<=120 && RGB(i,j,3) <= 120)  %Back Tag(Orange) on Firebird 
             colb(i,j)=255;
           
        end
        if(RGB(i,j,1) >=160 && RGB(i,j,2)>=160 && RGB(i,j,3)>=160)     % Multiple white  Objects
            coly(i,j)=255;
       
        end
            
    end
end

%imshow(RGB);
%preview(vid);
diff_im1 = bwareaopen(colw,50);

bw1 = bwlabel(diff_im1, 8);
stats = regionprops(bw1, 'basic');
hold on
%performing bounding box operation on green tag
for object = 1:length(stats)
            bb = stats(object).BoundingBox;
            
            area1 = stats(object).Area;
			
            if area1 > 50
                x = bb(1);
                y = bb(2);
                w = bb(3);
                h = bb(4);
                ct1=(bb(1)+bb(3)/2);
				ct2=(bb(2)+bb(4)/2);
				x2=ct1;y2=ct2;	
				rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
				%plot(ct1,ct2,'r*')
                
            end
            
            hold off
end
diff_im2 = bwareaopen(colb,50);
bw2 = bwlabel(diff_im2, 8);
stats = regionprops(bw2, 'basic');
hold on
%performing bounding box operation on orange tag
for object = 1:length(stats)
            bb = stats(object).BoundingBox;
          
            area2 = stats(object).Area;
			if area2 > 50
                x = bb(1);
                y = bb(2);
                w = bb(3);
                h = bb(4);
                ct3=(bb(1)+bb(3)/2);
				ct4=(bb(2)+bb(4)/2);
				rectangle('Position',bb,'EdgeColor','b','LineWidth',2)
				%plot(ct3,ct4,'b*')
                
            end
            
hold off
end

x1=ct3;y1=ct4;
%finding tobot center
rb_center_x = (ct1+ct3)/2;
rb_center_y = (ct2+ct4)/2;
%code for white object
diff_im3 = bwareaopen(coly,150);

bw3 = bwlabel(diff_im3, 8);
stats = regionprops(bw3, 'basic');
hold on
%imshow(diff_im3);
%performing bounding box operation on white objects
for object = 1:length(stats)
            bb = stats(object).BoundingBox;
         
            area3 = stats(object).Area;
			
            if (area3 > 300)
                x(object) = bb(1);
                y(object) = bb(2);
                w(object) = bb(3);
                h(object) = bb(4);
                ct5(object)=(bb(1)+bb(3)/2);
				ct6(object)=(bb(2)+bb(4)/2);
				x3(object)=ct5(object);
				y3(object)=ct6(object);	% this denotes center of white object
				rectangle('Position',bb,'EdgeColor','g','LineWidth',2)
				%plot(ct5,ct6,'g*')
              
            end
           
hold off
end
%white_object_center_x = ct5;
%white_object_center_y = ct6;
for i = 1:length(stats)
        d(i) = sqrt((rb_center_x - x(i))^2 + (rb_center_y - y(i))^2);
        
end
min = d(1);
k=1;
%finding minimum distance object
for j = 2:length(stats)
		if(d(j) < min)
		    min = d(j);
			k = j;
		end
end		
		
x3=x3(k);y3=y3(k);
%Algorithm to find angle for turning the firebird so that it aligns with
%the object.

a=(x2-x1)*(x3-x1)+(y2-y1)*(y3-y1);
b1=sqrt((x2-x1)^2+(y2-y1)^2);
b2=sqrt((x3-x1)^2+(y3-y1)^2);
b=b1*b2;
angl=acos(a/b);
angl1=angl*180/pi
angle=angl1*144/180;

m=(y2-y1)/(x2-x1);
c=-m*x1+y1;
if(m>0)
    if(x2-x1>0)
        if((-m*x3+y3-c)>0)
        f=3;
        end
        if((-m*x3+y3-c)<0)
        f=2;
        end
   
    else
       if((-m*x3+y3-c)>0)
        f=2;
        end
        if((-m*x3+y3-c)<0)
        f=3;
        end
    end
end
if(m<0)
    if(x1-x2>0)
        if((-m*x3+y3-c)>0)
        f=2;
        end
        if((-m*x3+y3-c)<0)
        f=3;
        end
   
    else
        if((-m*x3+y3-c)>0)
        f=3;
        end
        if((-m*x3+y3-c)<0)
        f=2;
        end
    end
end
% f denotes direction of robot 2-left, 3 - right
threshold = sqrt((x2-x3)^2+(y2-y3)^2);

 
  if (angl1 >=6 )
	  fwrite(s,angl1);
	  fwrite(s,f);
  elseif(threshold >=140)
      fwrite(s,1);
  elseif(angl1<6 && threshold <420)
      fwrite(s,0);
      flag = 1   
  end  
  
flushdata(vid);
	if (flag == 0)
        
       data_read = fread(s,1)
       while(data_read ~=50)
        data_read = fread(s,1);
       end
    end
end
 if(flag==1)
       fwrite(s,4);
 % Picking object      
       data_read1 = fread(s,1)
       while(data_read1 ~=51)
           data_read1 = fread(s,1)
       end   
       flag = 2  
       %stop(vid);
%stop(vid1);
flushdata(vid);
%fclose(s);
 end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  FLAG 2  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLACING    
if (flag ==2)
      now_flag_is = 2
% After picking object, trying to find out angle between new position of robot and given base station cordinate      
%flushdata(vid);  
RGB=getsnapshot(vid);
 
dim=size(RGB);%stores dimension of RGB matrix
%Color segmentation code
%coly=zeros(dim(1),dim(2));
colb = zeros(dim(1),dim(2));
colw = zeros(dim(1),dim(2));
x3=zeros(10,1);d=zeros(10,1);

for i=1:dim(1)
    for j=1:dim(2)
        if(RGB(i,j,1) <=110 && RGB(i,j,2) < 255 && RGB(i,j,2) > 120 && RGB(i,j,3) <= 140)  %Front Tag(Green) on Firebird
            colw(i,j)=255;
          % sumwi=sumwi+i;sumwj=sumwj+j;cw=cw+1;
        end
        if(RGB(i,j,1) >150 && RGB(i,j,2)<=120 && RGB(i,j,3) <= 120)  %Back Tag(Orange) on Firebird 
             colb(i,j)=255;
           % sumbi=sumbi+i;sumbj=sumbj+j;cb=cb+1;
        end
          
               
   end
end
%imshow(RGB);
%preview(vid);
diff_im1 = bwareaopen(colw,50);

bw1 = bwlabel(diff_im1, 8);
stats = regionprops(bw1, 'basic');
hold on

for object = 1:length(stats)
            bb = stats(object).BoundingBox;
            
            area1 = stats(object).Area;
			
            if area1 > 50
                x = bb(1);
                y = bb(2);
                w = bb(3);
                h = bb(4);
                ct1=(bb(1)+bb(3)/2);
				ct2=(bb(2)+bb(4)/2);
				x2=ct1;y2=ct2;	
				rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
				%plot(ct1,ct2,'r*')
                
            end
            
            hold off
end
diff_im2 = bwareaopen(colb,50);
bw2 = bwlabel(diff_im2, 8);
stats = regionprops(bw2, 'basic');
hold on

for object = 1:length(stats)
            bb = stats(object).BoundingBox;
          
            area2 = stats(object).Area
			if area2 > 50
                x = bb(1);
                y = bb(2);
                w = bb(3);
                h = bb(4);
                ct3=(bb(1)+bb(3)/2);
				ct4=(bb(2)+bb(4)/2);
				rectangle('Position',bb,'EdgeColor','b','LineWidth',2)
				%plot(ct3,ct4,'b*')
                
            end
            
hold off
end

x1=ct3;y1=ct4;
rb_center_x = (ct1+ct3)/2
rb_center_y = (ct2+ct4)/2
%code for white object

%white_object_center_x = ct5;
%white_object_center_y = ct6;
	
x3=331;y3=41;
%Algorithm to find angle for turning the firebird so that it aligns with
%the object.

a=(x2-x1)*(x3-x1)+(y2-y1)*(y3-y1);
b1=sqrt((x2-x1)^2+(y2-y1)^2);
b2=sqrt((x3-x1)^2+(y3-y1)^2);
b=b1*b2;
angl=acos(a/b);
angl2=angl*180/pi
angle=angl1*144/180;

m=(y2-y1)/(x2-x1);
c=-m*x1+y1;
if(m>0)
    if(x2-x1>0)
        if((-m*x3+y3-c)>0)
        f2=3;
        end
        if((-m*x3+y3-c)<0)
        f2=2;
        end
   
    else
       if((-m*x3+y3-c)>0)
        f2=2;
        end
        if((-m*x3+y3-c)<0)
        f2=3;
        end
    end
end
if(m<0)
    if(x1-x2>0)
        if((-m*x3+y3-c)>0)
        f2=2;
        end
        if((-m*x3+y3-c)<0)
        f2=3;
        end
   
    else
        if((-m*x3+y3-c)>0)
        f2=3;
        end
        if((-m*x3+y3-c)<0)
        f2=2;
        end
    end
end
   
% Distance between robot front tag and base station

threshold1 = sqrt((x2-x3)^2+(y2-y3)^2);

 
  if (angl2 >=10 )
	  fwrite(s,angl2);
	  fwrite(s,f2);
  elseif(threshold1 >=20)
      fwrite(s,1);
  elseif(angl2<10 && threshold1 <20)
      fwrite(s,0);
      flag = 3 
  end  
if(flag == 2)
data_read = fread(s,1);
while(data_read ~=50)
    data_read = freaf(s,1);
end
end
end
if (flag == 3)
    %now_flag=3
    flushdata(vid);
    fwrite(s,5);
% Placing object at base station    
  data_read3 = fread(s,1)
while(data_read3 ~=52)
    data_read3 = freaf(s,1);
end  
 fwrite(s,90);
 fwrite(s,2);
 %%placing has beeb done...turning 90 degree
 data_read = fread(s,1)
while(data_read ~=50)
    data_read = freaf(s,1);
end  
 flag = 0;
 flushdata(vid);
end



  %pause(2);
  
  
 end	
%fwrite(s,f);
%imshow(diff_im);

stop(vid);
%stop(vid1);
flushdata(vid);

  
fclose(s);


