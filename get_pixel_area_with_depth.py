import math
import cv2
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.path import Path
from numpy.core.fromnumeric import shape
import pyrealsense2 as rs

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from operator import itemgetter

def get_depth_at_pixel(depth_frame, pixel_x, pixel_y):
	"""
	Get the depth value at the desired image point
	Parameters:
	-----------
	depth_frame 	 : rs.frame()
						   The depth frame containing the depth information of the image coordinate
	pixel_x 	  	 	 : double
						   The x value of the image coordinate
	pixel_y 	  	 	 : double
							The y value of the image coordinate
	Return:
	----------
	depth value at the desired pixel
	"""
	return depth_frame.as_depth_frame().get_distance(round(pixel_x), round(pixel_y))

def get_points_in_polygon(depth_points):
    inside_point = []
    poly_points = [point[:2] for point in depth_points]
    min_x = min(poly_points,key=itemgetter(0))[0]
    max_x = max(poly_points,key=itemgetter(0))[0]

    min_y = min(poly_points,key=itemgetter(1))[1]
    max_y = max(poly_points,key=itemgetter(1))[1]

    polygon = Polygon(poly_points)
    for x in range(min_x, max_x+1):
        for y in range(min_y, max_y+1):
            if is_inside(x, y, polygon):
                inside_point.append([x,y])

    return inside_point

def get_area_with_points_in_polygon(depth_points):
    total_area = 0
    poly_points = [p[:2] for p in depth_points] # vertex

    min_x = min(poly_points,key=itemgetter(0))[0]
    max_x = max(poly_points,key=itemgetter(0))[0]
    min_y = min(poly_points,key=itemgetter(1))[1]
    max_y = max(poly_points,key=itemgetter(1))[1]
    """
    polygon = Polygon(poly_points)
    for x in range(min_x, max_x+1):
        for y in range(min_y, max_y+1):
            if is_inside(x, y, polygon):
                total_area += get_pixel_area((x,y))
    """
    x, y = np.meshgrid(np.arange(min_x, max_x+1), np.arange(min_y, max_y+1))
    x, y = x.flatten(), y.flatten()
    xy_points = np.vstack((x,y)).T
    polygon = Path(poly_points)
    grid = polygon.contains_points(xy_points)
    
    total_area = get_pixel_areas(xy_points[grid,:])
    
    return total_area


def is_inside(x, y, polygon):
    return Point(x,y).within(polygon)

def get_pixel_areas(pixel_points):
    total_area = 0
    for point in pixel_points:
        distance = get_depth_at_pixel(depth_frame, point[0], point[1])
        height_pixel = math.tan(math.pi * ((42/2)/180)) * (distance * 100) / 240
        width_pixel = math.tan(math.pi * ((69/2)/180)) * (distance * 100) / 320
        total_area += height_pixel * width_pixel

    return total_area

def get_pixel_area(pixel_point):
    distance = get_depth_at_pixel(depth_frame, pixel_point[0], pixel_point[1])
    height_pixel = math.tan(math.pi * ((42/2)/180)) * (distance * 100) / 240
    width_pixel = math.tan(math.pi * ((69/2)/180)) * (distance * 100) / 320

    return height_pixel * width_pixel



def mouse_callback(event, x, y, flags, param): 
    global point
    global mouse_click_count
    global depth_point
    global click_point
    #global center_point

    if event == cv2.EVENT_LBUTTONDOWN:
        print("Mouse Click Start")
        point = (x,y)
        click_point.append(point)
        depth = get_depth_at_pixel(depth_frame, x, y) # => depth = depth_frame.get_distance(x,y)
        
        print("마우스 click => (x,y): ({}, {}), depth: {}mm".format(x, y, (depth * 1000)))
        
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        #center_point = [depth_intrin.ppx, depth_intrin.ppy]
        depth_point.append(rs.rs2_deproject_pixel_to_point(depth_intrin, [point[0],point[1]], depth))
        print("{}th depth's point is {}".format(mouse_click_count + 1, depth_point[mouse_click_count]))
        #print("{}번째 실제 픽셀의 크기: {}cm^2".format(mouse_click_count + 1, get_pixel_area([x, y])))
        print()

        mouse_click_count += 1

    if event == cv2.EVENT_RBUTTONDOWN:
        if mouse_click_count <= 2:
            print("You click 2 times or less!! click more than 3 times!!")
            
        else:
            total_area = get_area_with_points_in_polygon(click_point)
            print("전체 넓이는 {}cm^2입니다".format(total_area))
            """
            area_of_polygon = calculate_area_of_polygon_with_point_location(depth_point)
            print("{}각형의 넓이는 {}cm^2 입니다".format(mouse_click_count, area_of_polygon * 10000))
            """

    if event == cv2.EVENT_MBUTTONDOWN:
        depth_point.clear()
        mouse_click_count = 0
        print("클릭한 좌표들 초기화")

# get are of polygon
def calculate_area_of_polygon_with_point_location(points):
    points.append(points[0])

    sum = 0
    for idx in range(1, len(points)):
        sum += points[idx-1][0] * points[idx][1]
        sum -= points[idx-1][1] * points[idx][0]

    return abs(sum / 2)


# get istance with two points
def calculate_with_two_points(first_point, second_point):
    x_distance = ((first_point[0] - second_point[0]) ** 2)
    y_distance = ((first_point[1] - second_point[1]) ** 2)
    square_x_y_distance = abs(x_distance + y_distance)

    z_distance = ((first_point[2] - second_point[2]) ** 2)
    
    return (square_x_y_distance + z_distance) ** (1/2)

# get area of square
def calculate_area_of_square_by_distances(width1, width2, height1, height2, diagonal):
    half_square1 = calculate_area_of_triangle_by_distances(width1, height1, diagonal)
    half_square2 = calculate_area_of_triangle_by_distances(width2, height2, diagonal)
    
    return half_square1 + half_square2

# get area of triangle
def calculate_area_of_triangle_by_distances(x, y, z):
    s = (x + y + z) / 2

    return (s * (s - x) * (s - y) * (s - z)) ** (1/2)





def convert_depth_pixel_to_metric_coordinate(depth, pixel_x, pixel_y, camera_intrinsics):
	"""
	Convert the depth and image point information to metric coordinates
	Parameters:
	-----------
	depth 	 	 	 : double
						   The depth value of the image point
	pixel_x 	  	 	 : double
						   The x value of the image coordinate
	pixel_y 	  	 	 : double
							The y value of the image coordinate
	camera_intrinsics : The intrinsic values of the imager in whose coordinate system the depth_frame is computed
	Return:
	----------
	X : double
		The x value in meters
	Y : double
		The y value in meters
	Z : double
		The z value in meters
	"""
	X = (pixel_x - camera_intrinsics.ppx)/camera_intrinsics.fx *depth
	Y = (pixel_y - camera_intrinsics.ppy)/camera_intrinsics.fy *depth
	return X, Y, depth



point = (250, 250)

depth_point = []
click_point = []
#global mouse_click_count
mouse_click_count = 0

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)



try:
    while True:
        global depth_frame

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()


        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue


        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())


        
        # Circle and Text for mouse click point
        cv2.circle(color_image, point, 4, (0, 0, 255))
        cv2.putText(color_image, "{}mm".format(get_depth_at_pixel(depth_frame, point[0], point[1]) * 1000), (point[0], point[1]), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0), 2)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

        cv2.setMouseCallback('RealSense', mouse_callback)
        
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

finally:
    # Stop streaming
    pipeline.stop()