"""
Copyright (c) 2026 Team CENGOT

Licensed under the MIT License. See the top-level LICENSE file for details.
"""

from picamera2 import Picamera2
import cv2
import numpy as np
import time
import serial


SERIAL_ENABLED = True
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD_RATE = 115200
SERIAL_TIMEOUT = 1
SERIAL_WRITE_TIMEOUT = 1
SERIAL_SEND_INTERVAL = 0.2
SERIAL_LINE_ENDING = "\n"

FRAME_WIDTH = 320
FRAME_HEIGHT = 240

CAMERA_FORMAT = "RGB888"
CAMERA_WARMUP_SECONDS = 1

MIN_CONTOUR_AREA = 250
CENTER_DEAD_ZONE = 35

ROI_TOP_RATIO = 0.25
ROI_BOTTOM_RATIO = 0.95
ROI_LEFT_RATIO = 0.10
ROI_RIGHT_RATIO = 0.90

LOOP_DELAY_SECONDS = 0.2

DEFAULT_OBSTACLE_COLOR = "none"
DEFAULT_OBSTACLE_POSITION = "none"
DEFAULT_STEERING_COMMAND = "go_straight"
DEFAULT_SPEED_COMMAND = "normal_speed"

RED_STEERING_COMMAND = "steer_left"
GREEN_STEERING_COMMAND = "steer_right"
OBSTACLE_SPEED_COMMAND = "slow_down"

SERIAL_COMMAND_TEMPLATE = "{steering_command},{speed_command},{obstacle_color},{obstacle_position},{obstacle_area},{obstacle_center_x},{obstacle_center_y}"

COLOR_RANGES = {
    "red": [
        ((0, 70, 50), (12, 255, 255)),
        ((165, 70, 50), (180, 255, 255))
    ],
    "green": [
        ((40, 60, 50), (88, 255, 255))
    ]
}


# start camera with fixed lightweight settings.
camera = Picamera2()

camera_config = camera.create_preview_configuration(
    main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": CAMERA_FORMAT}
)

camera.configure(camera_config)
camera.start()

time.sleep(CAMERA_WARMUP_SECONDS)


def create_serial_connection():
    # skip serial when disabled from config.
    if not SERIAL_ENABLED:
        return None

    try:
        connection = serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUD_RATE,
            timeout=SERIAL_TIMEOUT,
            write_timeout=SERIAL_WRITE_TIMEOUT
        )

        time.sleep(2)
        return connection

    except serial.SerialException as error:
        print(f"Serial connection error: {error}")
        return None


serial_connection = create_serial_connection()


def correct_camera_frame(frame):
    # convert frame to expected channel order.
    corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return corrected_frame


def get_region_of_interest(frame):
    # crop to the driving-relevant window.
    height, width, _ = frame.shape

    start_y = int(height * ROI_TOP_RATIO)
    end_y = int(height * ROI_BOTTOM_RATIO)
    start_x = int(width * ROI_LEFT_RATIO)
    end_x = int(width * ROI_RIGHT_RATIO)

    roi_frame = frame[start_y:end_y, start_x:end_x]

    return roi_frame, start_x, start_y


def create_color_mask(hsv_frame, hsv_ranges):
    # merge all hsv ranges for one color.
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

    # clean small noise and fill tiny gaps.
    total_mask = cv2.morphologyEx(total_mask, cv2.MORPH_OPEN, kernel)
    total_mask = cv2.morphologyEx(total_mask, cv2.MORPH_CLOSE, kernel)

    return total_mask


def find_largest_obstacle(mask, color_name, offset_x, offset_y):
    # pick the strongest valid contour.
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
    # run full detection pipeline on one frame.
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
    # prioritize the closest-looking target by area.
    if not obstacles:
        return None

    primary_obstacle = max(obstacles, key=lambda item: item["area"])

    return primary_obstacle


def calculate_obstacle_position(obstacle):
    # map obstacle center to left/center/right.
    frame_center_x = FRAME_WIDTH // 2
    obstacle_center_x = obstacle["center_x"]

    if obstacle_center_x < frame_center_x - CENTER_DEAD_ZONE:
        return "left"

    if obstacle_center_x > frame_center_x + CENTER_DEAD_ZONE:
        return "right"

    return "center"


def generate_driving_decision(obstacle):
    # build safe defaults when nothing is detected.
    if obstacle is None:
        return {
            "obstacle_color": DEFAULT_OBSTACLE_COLOR,
            "obstacle_position": DEFAULT_OBSTACLE_POSITION,
            "steering_command": DEFAULT_STEERING_COMMAND,
            "speed_command": DEFAULT_SPEED_COMMAND,
            "obstacle_area": 0,
            "obstacle_center_x": 0,
            "obstacle_center_y": 0
        }

    obstacle_color = obstacle["color"]
    obstacle_position = calculate_obstacle_position(obstacle)

    # map color to steering behavior.
    if obstacle_color == "red":
        steering_command = RED_STEERING_COMMAND
        speed_command = OBSTACLE_SPEED_COMMAND

    elif obstacle_color == "green":
        steering_command = GREEN_STEERING_COMMAND
        speed_command = OBSTACLE_SPEED_COMMAND

    else:
        steering_command = DEFAULT_STEERING_COMMAND
        speed_command = DEFAULT_SPEED_COMMAND

    decision = {
        "obstacle_color": obstacle_color,
        "obstacle_position": obstacle_position,
        "steering_command": steering_command,
        "speed_command": speed_command,
        "obstacle_area": int(obstacle["area"]),
        "obstacle_center_x": obstacle["center_x"],
        "obstacle_center_y": obstacle["center_y"]
    }

    return decision


def create_serial_message(decision):
    # format one csv packet for vehicle.cpp.
    message = SERIAL_COMMAND_TEMPLATE.format(
        steering_command=decision["steering_command"],
        speed_command=decision["speed_command"],
        obstacle_color=decision["obstacle_color"],
        obstacle_position=decision["obstacle_position"],
        obstacle_area=decision["obstacle_area"],
        obstacle_center_x=decision["obstacle_center_x"],
        obstacle_center_y=decision["obstacle_center_y"]
    )

    return message + SERIAL_LINE_ENDING


def send_decision_to_serial(connection, decision):
    # send decision only when serial is ready.
    if connection is None:
        return False

    if not connection.is_open:
        return False

    message = create_serial_message(decision)

    try:
        connection.write(message.encode("utf-8"))
        return True

    except serial.SerialException as error:
        print(f"Serial write error: {error}")
        return False


def print_detection_result(obstacles, primary_obstacle, decision, serial_sent):
    # print one compact debug frame.
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
    print(f"Obstacle area: {decision['obstacle_area']}")
    print(f"Obstacle center x: {decision['obstacle_center_x']}")
    print(f"Obstacle center y: {decision['obstacle_center_y']}")
    print("----------------------------------")
    print(f"Serial enabled: {SERIAL_ENABLED}")
    print(f"Serial port: {SERIAL_PORT}")
    print(f"Serial baud rate: {SERIAL_BAUD_RATE}")
    print(f"Serial sent: {serial_sent}")
    print(f"Serial message: {create_serial_message(decision).strip()}")


try:
    # throttle serial sending independent of loop delay.
    last_serial_send_time = 0

    while True:
        # capture frame and compute latest decision.
        frame = camera.capture_array()

        obstacles = detect_obstacles(frame)

        primary_obstacle = choose_primary_obstacle(obstacles)

        decision = generate_driving_decision(primary_obstacle)

        current_time = time.time()

        serial_sent = False

        # send at a fixed interval to avoid serial flooding.
        if current_time - last_serial_send_time >= SERIAL_SEND_INTERVAL:
            serial_sent = send_decision_to_serial(serial_connection, decision)
            last_serial_send_time = current_time

        print_detection_result(obstacles, primary_obstacle, decision, serial_sent)

        time.sleep(LOOP_DELAY_SECONDS)

except KeyboardInterrupt:
    print("\nProgram stopped by user.")

finally:
    # close camera and serial cleanly.
    camera.stop()

    if serial_connection is not None and serial_connection.is_open:
        serial_connection.close()

    print("Camera stopped.")
    print("Serial connection closed.")