import numpy as np
import cv2


def RGB_color_selection(image):
    """
    Apply color selection to RGB images to blackout everything except for white and yellow lane lines.
        Parameters:
            image: An np.array compatible with plt.imshow.
    """
    #White color mask
    lower_threshold = np.uint8([200, 200, 200]) # 200 200 200
    upper_threshold = np.uint8([255, 255, 255]) # 255 255 255
    white_mask = cv2.inRange(image, lower_threshold, upper_threshold)

    #Black color mask
    lower_threshold = np.uint8([0, 0, 0]) # 200 200 200
    upper_threshold = np.uint8([50,50, 50]) # 255 255 255
    black_mask = cv2.inRange(image, lower_threshold, upper_threshold)
    
    #Yellow color mask
    lower_threshold = np.uint8([175, 175,   0])
    upper_threshold = np.uint8([255, 255, 255])
    yellow_mask = cv2.inRange(image, lower_threshold, upper_threshold)
    
    #Combine white and yellow masks
    # mask = cv2.bitwise_or(white_mask,yellow_mask)
    mask = black_mask
    masked_image = cv2.bitwise_and(image, image, mask = mask)
    
    return masked_image

def convert_hsv(image):
    """
    Convert RGB images to HSV.
        Parameters:
            image: An np.array compatible with plt.imshow.
    """
    return cv2.cvtColor(image, cv2.COLOR_RGB2HSV)


def HSV_color_selection(image):
    """
    Apply color selection to the HSV images to blackout everything except for white and yellow lane lines.
        Parameters:
            image: An np.array compatible with plt.imshow.
    """
    #Convert the input image to HSV
    converted_image = convert_hsv(image)
    
    #White color mask
    lower_threshold = np.uint8([0, 0, 210])
    upper_threshold = np.uint8([255, 30, 255])
    white_mask = cv2.inRange(converted_image, lower_threshold, upper_threshold)
    
    #Yellow color mask
    lower_threshold = np.uint8([18, 80, 80])
    upper_threshold = np.uint8([30, 255, 255])
    yellow_mask = cv2.inRange(converted_image, lower_threshold, upper_threshold)
    
    #Combine white and yellow masks
    mask = cv2.bitwise_or(white_mask, yellow_mask)
    masked_image = cv2.bitwise_and(image, image, mask = mask)
    
    return masked_image


def convert_hsl(image):
    """
    Convert RGB images to HSL.
        Parameters:
            image: An np.array compatible with plt.imshow.
    """
    return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)


def HSL_color_selection(image):
    """
    Apply color selection to the HSL images to blackout everything except for white and yellow lane lines.
        Parameters:
            image: An np.array compatible with plt.imshow.
    """
    #Convert the input image to HSL
    converted_image = convert_hsl(image)
    
    #White color mask
    lower_threshold = np.uint8([0, 200, 0])
    upper_threshold = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(converted_image, lower_threshold, upper_threshold)
    
    #Yellow color mask
    lower_threshold = np.uint8([10, 0, 100])
    upper_threshold = np.uint8([40, 255, 255])
    yellow_mask = cv2.inRange(converted_image, lower_threshold, upper_threshold)
    
    # Black color mask
    # HSL에서 검은색은 주로 낮은 명도를 가짐. 색상과 채도는 넓은 범위 가능.
    # lower_black_threshold = np.uint8([0, 0, 0])
    # upper_black_threshold = np.uint8([255, 255, 80]) # 명도를 낮게 설정하여 검은색 영역 선택
    # black_mask = cv2.inRange(converted_image, lower_black_threshold, upper_black_threshold)
    
    #Combine white and yellow masks
    # mask = cv2.bitwise_or(white_mask, black_mask)
    mask = white_mask
    masked_image = cv2.bitwise_and(image, image, mask = mask)
    
    return masked_image


def gray_scale(image):
    """
    Convert images to gray scale.
        Parameters:
            image: An np.array compatible with plt.imshow.
    """
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)


def gaussian_smoothing(image, kernel_size = 13):
    """
    Apply Gaussian filter to the input image.
        Parameters:
            image: An np.array compatible with plt.imshow.
            kernel_size (Default = 13): The size of the Gaussian kernel will affect the performance of the detector.
            It must be an odd number (3, 5, 7, ...).
    """
    return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)


def canny_detector(image, low_threshold = 50, high_threshold = 150):
    """
    Apply Canny Edge Detection algorithm to the input image.
        Parameters:
            image: An np.array compatible with plt.imshow.
            low_threshold (Default = 50).
            high_threshold (Default = 150).
    """
    return cv2.Canny(image, low_threshold, high_threshold)


def region_selection(image):
    """
    Determine and cut the region of interest in the input image.
        Parameters:
            image: An np.array compatible with plt.imshow.
    """
    mask = np.zeros_like(image)   
    #Defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(image.shape) > 2:
        channel_count = image.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
    #We could have used fixed numbers as the vertices of the polygon,
    #but they will not be applicable to images with different dimesnions.
    rows, cols = image.shape[:2]
    bottom_left  = [cols * 0.0, rows * 1.0] # cols * 0.1, rows * 0.95
    top_left     = [cols * 0.0, rows * 0.5] # cols * 0.4, rows * 0.6
    bottom_right = [cols * 1.0, rows * 1.0] # cols * 0.9, rows * 0.95
    top_right    = [cols * 1.0, rows * 0.5] # cols * 0.6, rows * 0.6
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


def hough_transform(image):
    """
    Determine and cut the region of interest in the input image.
        Parameters:
            image: The output of a Canny transform.
    """
    rho = 1              #Distance resolution of the accumulator in pixels.
    theta = np.pi/180    #Angle resolution of the accumulator in radians.
    threshold = 50       #Only lines that are greater than threshold will be returned.
    minLineLength = 40   #Line segments shorter than that are rejected.
    maxLineGap = 300     #Maximum allowed gap between points on the same line to link them
    return cv2.HoughLinesP(image, rho = rho, theta = theta, threshold = threshold,
                           minLineLength = minLineLength, maxLineGap = maxLineGap)


def draw_lines(image, lines, color = [255, 0, 0], thickness = 2):
    """
    Draw lines onto the input image.
        Parameters:
            image: An np.array compatible with plt.imshow.
            lines: The lines we want to draw.
            color (Default = red): Line color.
            thickness (Default = 2): Line thickness.
    """
    image = np.copy(image)
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(image, (x1, y1), (x2, y2), color, thickness)
    return image


def average_slope_intercept(lines):
    """
    Find the slope and intercept of the left and right lanes of each image.
        Parameters:
            lines: The output lines from Hough Transform.
    """
    left_lines    = [] #(slope, intercept)
    left_weights  = [] #(length,)
    right_lines   = [] #(slope, intercept)
    right_weights = [] #(length,)
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            if x1 == x2:
                continue
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))
            if slope < 0:
                left_lines.append((slope, intercept))
                left_weights.append((length))
            else:
                right_lines.append((slope, intercept))
                right_weights.append((length))
    left_lane  = np.dot(left_weights,  left_lines) / np.sum(left_weights)  if len(left_weights) > 0 else None
    right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None
    return left_lane, right_lane


def pixel_points(y1, y2, line):
    """
    Converts the slope and intercept of each line into pixel points.
        Parameters:
            y1: y-value of the line's starting point.
            y2: y-value of the line's end point.
            line: The slope and intercept of the line.
    """
    if line is None:
        return None
    slope, intercept = line
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    y1 = int(y1)
    y2 = int(y2)
    return ((x1, y1), (x2, y2))


def lane_lines(image, lines):
    """
    Create full lenght lines from pixel points.
        Parameters:
            image: The input test image.
            lines: The output lines from Hough Transform.
    """
    try:
        left_lane, right_lane = average_slope_intercept(lines)
        y1 = image.shape[0]
        y2 = y1 * 0.6
        # print(y1)
        left_line  = pixel_points(y1, y2, left_lane)
        right_line = pixel_points(y1, y2, right_lane)
    except:
        left_line = None
        right_line = None
        # print("None")
        
    
    return left_line, right_line

    
def draw_lane_lines(image, lines, color=[255, 0, 0], thickness=12):
    """
    Draw lines onto the input image.
        Parameters:
            image: The input test image.
            lines: The output lines from Hough Transform.
            color (Default = red): Line color.
            thickness (Default = 12): Line thickness. 
    """
    line_image = np.zeros_like(image)
    for line in lines:
        if line is not None:
            cv2.line(line_image, *line,  color, thickness)
    return cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)


def frame_processor(image):
    """
    Process the input frame to detect lane lines.
        Parameters:
            image: Single video frame.
    """
    color_mid = (0,255,0)
    color = (255,255,0)
    # mid_low = (480,540)
    # mid_high = (480,0)
    mid_low = (320,480)
    mid_high = (320,0)
    
    color_select = HSL_color_selection(image)
    # color_select = RGB_color_selection(image)
    gray         = gray_scale(color_select)
    smooth       = gaussian_smoothing(gray)
    edges        = canny_detector(smooth)
    region       = region_selection(edges)
    hough        = hough_transform(region)
    mid_point    = get_line_mid(image,hough)
    result       = draw_lane_lines(image, lane_lines(image, hough))
    
    cv2.line(result, pt1=mid_low, pt2=mid_high, color=color_mid, thickness=3)
    cv2.line(result, pt1=(mid_point,0), pt2=(mid_point,540), color=color, thickness=3)
    
    return result, mid_point #result, mid_point

def get_line_mid(image,hough):
    lane_width = 320
    try :
        # look y2 value -> line's end point
        left_line, right_line =lane_lines(image,hough)
        # x_mid, y_mid = lane_lines(image,hough)

        # print(left_line, right_line)
        if left_line == None:
            x_mid = int(right_line[1][0] - (lane_width / 2))
            # print("left_line is none")
        elif right_line == None:
            x_mid = int(left_line[1][0] + (lane_width / 2))
            # print("right_line is none")
        else:
            x_mid = int((left_line[1][0] + right_line[1][0])/2)

        # x_mid = int((left_line[1][0] + right_line[1][0])/2)
        # y_mid = int((left_line[1][1] + right_line[1][1])/2)
        
        if x_mid > 960 or x_mid < 0:
            # x_mid = None
            x_mid = -1
        
    except :
        # x_mid = None
        x_mid = -1
        # y_mid = None
        
    return x_mid
