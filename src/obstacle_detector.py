"""
Copyright (c) 2026 Team CENGOT

Licensed under the MIT License. See the top-level LICENSE file for details.
"""

from picamera2 import Picamera2
import cv2
import numpy as np
import time






camera = Picamera2()

camera_config = camera.create_preview_configuration(
    main={"size": (320, 240), "format": "RGB888"}
)

camera.configure(camera_config)
camera.start()

time.sleep(1)





# detection settings
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

MIN_CONTOUR_AREA = 250
CENTER_DEAD_ZONE = 35


# only the lower-middle part of the image is used to reduce false detections from background objects
ROI_TOP_RATIO = 0.25
ROI_BOTTOM_RATIO = 0.95
ROI_LEFT_RATIO = 0.10
ROI_RIGHT_RATIO = 0.90




COLOR_RANGES = {
    "red": [
        ((0, 70, 50), (12, 255, 255)),
        ((165, 70, 50), (180, 255, 255))
    ],
    "green": [
        ((40, 60, 50), (88, 255, 255))
    ]
}



# utility functions

def correct_camera_frame(frame):


    corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return corrected_frame


def get_region_of_interest(frame):


    height, width, _ = frame.shape

    start_y = int(height * ROI_TOP_RATIO)
    end_y = int(height * ROI_BOTTOM_RATIO)
    start_x = int(width * ROI_LEFT_RATIO)
    end_x = int(width * ROI_RIGHT_RATIO)

    roi_frame = frame[start_y:end_y, start_x:end_x]

    return roi_frame, start_x, start_y


def create_color_mask(hsv_frame, hsv_ranges):


    total_mask = None

    for lower_bound, upper_bound in hsv_ranges:
        lower_array = np.array(lower_bound, dtype=np.uint8)
        upper_array = np.array(upper_bound, dtype=np.uint8)

        mask = cv2.inRange(hsv_frame, lower_array, upper_array)

        if total_mask is None:
            total_mask = mask
        else:
            total_mask = cv2.bitwise_or(total_mask, mask)

    kernel = np.ones((5, 5), np.uint8)

    total_mask = cv2.morphologyEx(total_mask, cv2.MORPH_OPEN, kernel)
    total_mask = cv2.morphologyEx(total_mask, cv2.MORPH_CLOSE, kernel)

    return total_mask


def find_largest_obstacle(mask, color_name, offset_x, offset_y):


    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    largest_contour = max(contours, key=cv2.contourArea)
    contour_area = cv2.contourArea(largest_contour)

    if contour_area < MIN_CONTOUR_AREA:
        return None

    x, y, width, height = cv2.boundingRect(largest_contour)

    center_x = offset_x + x + width // 2
    center_y = offset_y + y + height // 2

    obstacle = {
        "color": color_name,
        "area": contour_area,
        "center_x": center_x,
        "center_y": center_y,
        "width": width,
        "height": height,
        "box_x": offset_x + x,
        "box_y": offset_y + y
    }

    return obstacle


def detect_obstacles(frame):
    """
    detects red and green colors, returns a list of detected obstacles
    """

    corrected_frame = correct_camera_frame(frame)

    roi_frame, offset_x, offset_y = get_region_of_interest(corrected_frame)

    hsv_frame = cv2.cvtColor(roi_frame, cv2.COLOR_RGB2HSV)

    detected_obstacles = []

    for color_name, hsv_ranges in COLOR_RANGES.items():
        mask = create_color_mask(hsv_frame, hsv_ranges)

        obstacle = find_largest_obstacle(
            mask=mask,
            color_name=color_name,
            offset_x=offset_x,
            offset_y=offset_y
        )

        if obstacle is not None:
            detected_obstacles.append(obstacle)

    return detected_obstacles


def choose_primary_obstacle(obstacles):
    """
    the obstacle with largest visible are is selected if more than one obstacle is visible
    """

    if not obstacles:
        return None

    primary_obstacle = max(obstacles, key=lambda item: item["area"])

    return primary_obstacle


def calculate_obstacle_position(obstacle):
    """
    determines whether the obstacle is on the left,center or right side
    """

    frame_center_x = FRAME_WIDTH // 2
    obstacle_center_x = obstacle["center_x"]

    if obstacle_center_x < frame_center_x - CENTER_DEAD_ZONE:
        return "left"

    if obstacle_center_x > frame_center_x + CENTER_DEAD_ZONE:
        return "right"

    return "center"


def generate_driving_decision(obstacle):
    """
    Converts detected obstacle color into a driving decision.
    """

    if obstacle is None:
        return {
            "obstacle_color": "none",
            "obstacle_position": "none",
            "steering_command": "go_straight",
            "speed_command": "normal_speed"
        }

    obstacle_color = obstacle["color"]
    obstacle_position = calculate_obstacle_position(obstacle)

    if obstacle_color == "red":
        steering_command = "steer_left"
        speed_command = "slow_down"

    elif obstacle_color == "green":
        steering_command = "steer_right"
        speed_command = "slow_down"

    else:
        steering_command = "go_straight"
        speed_command = "normal_speed"

    decision = {
        "obstacle_color": obstacle_color,
        "obstacle_position": obstacle_position,
        "steering_command": steering_command,
        "speed_command": speed_command
    }

    return decision


def print_detection_result(obstacles, primary_obstacle, decision):
    """
    Prints obstacle detection and driving decision data to the terminal.
    """

    print("\033c", end="")
    print("WRO Color Based Obstacle Detection")
    print("----------------------------------")

    if not obstacles:
        print("Detected obstacles: none")
    else:
        print(f"Detected obstacles: {len(obstacles)}")

        for index, obstacle in enumerate(obstacles, start=1):
            print(
                f"{index}. Color: {obstacle['color']} | "
                f"Area: {int(obstacle['area'])} | "
                f"Center: ({obstacle['center_x']}, {obstacle['center_y']})"
            )

    print("----------------------------------")

    if primary_obstacle is None:
        print("Primary obstacle: none")
    else:
        print(f"Primary obstacle color: {primary_obstacle['color']}")
        print(f"Primary obstacle area: {int(primary_obstacle['area'])}")
        print(
            f"Primary obstacle center: "
            f"({primary_obstacle['center_x']}, {primary_obstacle['center_y']})"
        )

    print("----------------------------------")
    print(f"Obstacle color: {decision['obstacle_color']}")
    print(f"Obstacle position: {decision['obstacle_position']}")
    print(f"Steering command: {decision['steering_command']}")
    print(f"Speed command: {decision['speed_command']}")



#main loop
try:
    while True:
        frame = camera.capture_array()

        obstacles = detect_obstacles(frame)

        primary_obstacle = choose_primary_obstacle(obstacles)

        decision = generate_driving_decision(primary_obstacle)

        print_detection_result(obstacles, primary_obstacle, decision)

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nProgram stopped by user.")

finally:
    camera.stop()
    print("Camera stopped.")