import cv2 as cv;
import numpy as np
from matplotlib import pyplot as plt

read = cv.imread("test_image.jpg")

lane_img = np.copy(read);

def canny_image(image):
	gray = cv.cvtColor(image,cv.COLOR_RGB2GRAY)
	blur = cv.GaussianBlur(gray,(5,5),0)
	canny = cv.Canny(blur,50,150)
	return canny

def region_of_interest(image):
	height = image.shape[0]
	triangle = np.array([[(200,height),(1100,height),(550,250)]])
	mask = np.zeros_like(image)
	cv.fillPoly(mask,triangle,255)
	masked_image = cv.bitwise_and(image,mask)
	return masked_image;

def display_line(image,lines):
	line_img = np.copy(image);
	if lines is not None:
		for i in lines:			
			x1,y1,x2,y2 = i		
			cv.line(line_img,(x1,y1),(x2,y2),(255,0,0),10)

	return line_img

def houghLines(image):
	# arg1 input image
	# arg2 and arg3 (p and angle) accuracies
	# arg4 threshold value 
	# arg5 
	# arg6 minLineLength = min length of the line
	# arg7 maxLineGap max allowed gap between line segments to treat them as single line
	return cv.HoughLinesP(image,2,np.pi/180,100,np.array([]),minLineLength=40,maxLineGap=10)

# making coordinates
def define_coordinates(image,parameters):
	slope,intercept = parameters;
	# image.shape[0] is the value of y
	y1 = image.shape[0]
	y2 = int(y1 * (3/5))	
	# y = slope * x + intercept
	# x = (y - intercept)/slope
	x1 = int((y1 - intercept) / slope)
	x2 = int((y2 - intercept) / slope)

	return np.array([x1,y1,x2,y2])
# getting average slope 
def avgerage_slope_intercept(image,line):
	left_fit = []
	right_fit = []

	for i in line:
		x1,y1,x2,y2 = i.reshape(4)		
		# poly fit accepts the data set which polynomial function 
		# of any degree and returns array of the coefficients
		# that minimize the squared error
		# for i in range(k):
		#    sum += np.array(p(x) - y) ** 2
		parameters = np.polyfit((x1,x2),(y1,y2),1)
		slope,intercept = parameters
		if slope < 0:
			left_fit.append((slope,intercept))
		else:
			right_fit.append((slope,intercept))
	left_fit_average = np.average(left_fit,axis=0)
	right_fit_average = np.average(right_fit,axis=0)
	left_line = define_coordinates(image,left_fit_average)
	right_line = define_coordinates(image,right_fit_average)
	
	return np.array([left_line,right_line])

video = cv.VideoCapture('test2.mp4')
while video.isOpened():
	_,frame = video.read()
	canny = canny_image(frame)
	masked_image = region_of_interest(canny)
	line = houghLines(masked_image);
	avg_slope = avgerage_slope_intercept(frame,line)
	line_image = display_line(frame,avg_slope)
	cv.imshow("Line image: ",line_image)
	# cv2.waitKey(0)
	if cv.waitKey(1) & 0xFF == ord('q'):
		break;
video.release();
cv.destroyAllWindows();
