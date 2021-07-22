import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs

# 마우스 클릭 이벤트
def mouse_callback(event, x, y, flags, param): 
    global point # 마우스 클릭을 했을 때 화면의 좌표
    global mouse_click_count # 마우스 클릭 횟수
    global depth_point # 마우스 클릭을 했을 때 실제 좌표인 (x, y, z)를 담은 배열

    # 좌버튼 클릭시
    if event == cv2.EVENT_LBUTTONDOWN:
        point = (x,y)
        depth = get_depth_at_pixel(depth_frame, x, y) # => depth = depth_frame.get_distance(x,y)
        
        print("마우스 click => (x,y): ({}, {}), depth: {}mm".format(x, y, (depth * 1000)))
        
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        depth_point.append(rs.rs2_deproject_pixel_to_point(depth_intrin, [point[0],point[1]], depth))
        print("{}th depth's point is {}".format(mouse_click_count + 1, depth_point[mouse_click_count]))
        print()

        mouse_click_count += 1

        """
        # square need four point
        if mouse_click_count == 4:
            print("Four Clicks Success!!")

            width1 = calculate_with_two_points(depth_point[0], depth_point[1])
            width2 = calculate_with_two_points(depth_point[2], depth_point[3])
            
            height1 = calculate_with_two_points(depth_point[0], depth_point[2])
            height2 = calculate_with_two_points(depth_point[1], depth_point[3])
            
            diagonal = calculate_with_two_points(depth_point[1], depth_point[2])

            print("width1 is {}, width2 is {}".format(width1 * 100, width2 * 100))
            print("height1 is {}, height2 is {}".format(height1 * 100, height2 * 100))
            print("diagonal is {}".format(diagonal * 100))
            print()

            area_of_square = calculate_area_of_square_by_distances(width1, width2, height1, height2, diagonal)

            print("사각형의 넓이는: {}cm^2".format(area_of_square * 10000))
        """

    if event == cv2.EVENT_RBUTTONDOWN:
        if mouse_click_count <= 2:
            print("You click 2 times or less!! click more than 3 times!!")
            
        else:
            area_of_polygon = calculate_area_of_polygon_with_point_location(depth_point)
            print("{}각형의 넓이는 {}cm^2 입니다".format(mouse_click_count, area_of_polygon * 10000))

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
        
        # 마우스로 클릭한 좌표에 원 + 텍스트
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