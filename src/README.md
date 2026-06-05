Control software
====

## Obstacle Detecting

The obstacle detecting module is used to identify red and green traffic signs on the track by using the Raspberry Pi Camera Module 3 and OpenCV. In the obstacle round, the vehicle must react to colored traffic signs: the red traffic sign represents the right side of the lane, while the green traffic sign represents the left side. Therefore, the software converts the detected color into a basic steering decision for the autonomous vehicle.

The camera is initialized with `Picamera2` at a resolution of `320x240`. This resolution is preferred because it is lightweight enough for the Raspberry Pi Zero 2 W and still provides enough visual information for color-based obstacle detection.

```python
camera_config = camera.create_preview_configuration(
    main={"size": (320, 240), "format": "RGB888"}
)
```

The software only processes the lower-middle part of the image by using a Region of Interest. This reduces false detections from background objects and allows the vehicle to focus on the area where obstacles are expected to appear.

```python
ROI_TOP_RATIO = 0.25
ROI_BOTTOM_RATIO = 0.95
ROI_LEFT_RATIO = 0.10
ROI_RIGHT_RATIO = 0.90
```

The detection algorithm uses the HSV color space instead of directly using RGB values. HSV makes color filtering more stable under different lighting conditions. In this project, only red and green color ranges are defined because these colors represent the traffic signs used in the obstacle game.

```python
COLOR_RANGES = {
    "red": [
        ((0, 70, 50), (12, 255, 255)),
        ((165, 70, 50), (180, 255, 255))
    ],
    "green": [
        ((40, 60, 50), (88, 255, 255))
    ]
}
```

For each color, the program creates a binary mask with `cv2.inRange()`. White pixels in the mask represent the selected color, while black pixels represent the rest of the image. Morphological opening and closing operations are then applied to remove small noise and make the detected obstacle area cleaner.

```python
mask = cv2.inRange(hsv_frame, lower_array, upper_array)

total_mask = cv2.morphologyEx(total_mask, cv2.MORPH_OPEN, kernel)
total_mask = cv2.morphologyEx(total_mask, cv2.MORPH_CLOSE, kernel)
```

After the mask is created, contours are found with OpenCV. The largest contour is selected as the obstacle candidate. Very small contours are ignored with `MIN_CONTOUR_AREA`, which prevents random pixels or reflections from being accepted as obstacles.

```python
MIN_CONTOUR_AREA = 250

largest_contour = max(contours, key=cv2.contourArea)
contour_area = cv2.contourArea(largest_contour)
```

The program calculates the center position and size of the detected obstacle by using a bounding rectangle. This information is stored in a dictionary that includes the obstacle color, area, center point, width, and height.

```python
obstacle = {
    "color": color_name,
    "area": contour_area,
    "center_x": center_x,
    "center_y": center_y,
    "width": width,
    "height": height
}
```

If more than one obstacle is visible, the software selects the obstacle with the largest visible area as the primary obstacle. This is based on the assumption that the larger object in the camera image is usually closer and more important for the vehicle’s next movement.

```python
primary_obstacle = max(obstacles, key=lambda item: item["area"])
```

Finally, the detected obstacle color is converted into a driving decision. If a red obstacle is detected, the vehicle generates a `steer_left` command. If a green obstacle is detected, the vehicle generates a `steer_right` command. If no obstacle is detected, the vehicle continues with `go_straight`.

```python
if obstacle_color == "red":
    steering_command = "steer_left"
    speed_command = "slow_down"

elif obstacle_color == "green":
    steering_command = "steer_right"
    speed_command = "slow_down"
```

The current version prints the detected obstacle data and generated driving command to the terminal. This makes testing easier before connecting the obstacle detection output to the actual motor and steering control system.


### Hardware Components Used for Color Detecting

| Component                        | Image                                                   | Purpose                                                                                                                                                                            |
| -------------------------------- | ------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Raspberry Pi Zero 2 W            | <img src="raspberry-pi-zero-2w.jpeg" width="180"> | The Raspberry Pi Zero 2 W runs the Python control software and processes the camera frames using OpenCV. It acts as the main onboard computer of the vehicle.                      |
| Raspberry Pi Camera Module 3     | <img src="camera-module-3.jpeg" width="180">      | The Camera Module 3 captures real-time images from the environment. These images are used by the color detection algorithm to identify objects or markers in front of the vehicle. |
| Raspberry Pi Camera Ribbon Cable | <img src="camera-ribbon-cable.jpeg" width="180">  | The camera ribbon cable connects the Camera Module 3 to the Raspberry Pi Zero 2 W. It transfers image data from the camera to the Raspberry Pi for processing.                     |


This module is an important part of the robot car’s control software because it converts visual information into simple color-based data. This detected color can later be connected to the decision-making part of the vehicle, allowing the car to react to colored signs, objects, or track markers.
