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


color_ranges = {
    "red": [
        ((0, 70, 50), (12, 255, 255)),
        ((165, 70, 50), (180, 255, 255))
    ],
    "orange": [
        ((13, 80, 60), (24, 255, 255))
    ],
    "yellow": [
        ((25, 80, 70), (38, 255, 255))
    ],
    "green": [
        ((40, 60, 50), (88, 255, 255))
    ],
    "blue": [
        ((95, 70, 50), (130, 255, 255))
    ],
    "black": [
        ((0, 0, 0), (180, 255, 45))
    ],
    "white": [
        ((0, 0, 180), (180, 60, 255))
    ]
}


def get_center_roi(frame):
    height, width, _ = frame.shape

    start_x = int(width * 0.25)
    end_x = int(width * 0.75)
    start_y = int(height * 0.25)
    end_y = int(height * 0.75)

    return frame[start_y:end_y, start_x:end_x]


def get_color_pixel_count(hsv_frame, hsv_ranges):
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

    return cv2.countNonZero(total_mask)


def detect_color(frame):
    # Kırmızı blue çıkıyorsa mesele burada.
    # Bu satır kırmızı-mavi kanal tersliğini düzeltir.
    corrected_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    roi_frame = get_center_roi(corrected_frame)

    hsv_frame = cv2.cvtColor(roi_frame, cv2.COLOR_RGB2HSV)

    color_counts = {}
    detected_color = "none"
    highest_count = 0

    for color_name, hsv_ranges in color_ranges.items():
        pixel_count = get_color_pixel_count(hsv_frame, hsv_ranges)
        color_counts[color_name] = pixel_count

        if pixel_count > highest_count:
            highest_count = pixel_count
            detected_color = color_name

    minimum_pixel_count = 400

    if highest_count < minimum_pixel_count:
        detected_color = "none"

    return detected_color, highest_count, color_counts


try:
    while True:
        frame = camera.capture_array()

        detected_color, highest_count, color_counts = detect_color(frame)

        print("\033c", end="")
        print("Color detector running...")
        print("Show the object to the CENTER of the camera.")
        print("----------------------------------------")
        print(f"Detected color: {detected_color}")
        print(f"Highest pixel count: {highest_count}")
        print("----------------------------------------")

        for color_name, pixel_count in color_counts.items():
            print(f"{color_name}: {pixel_count}")

        time.sleep(0.3)

except KeyboardInterrupt:
    print("\nProgram stopped by user.")

finally:
    camera.stop()
    print("Camera stopped.")
