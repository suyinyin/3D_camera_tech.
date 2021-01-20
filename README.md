# object_tracking_3Dcamera

Object tracking for soft arm through Intel realsense.

>- `3Drendering.h` was used to render 3d graphic on the monitor.
>- `balldection_kalman.cpp` was used to track ball by implemented Kalman algorithm to optimize this process.
>- `Balltracking.cpp` and `Balltracking.h` was coined to track ball and implemented thorough Hough transform and contours detection algorithms to find the objects from segmentation images.
>- `camera_calibration.cpp` was realized by Opencv lib and it is a 2D camera calibration approach using chessboard.
>- `find_hsv.cpp` wad created to find the HSV range of the tracking object.
>- `main.cpp` was the main function to capture 4 balls with different covers to identify the position and orientation of the soft arm end-effector in reall time. 
>- `object_detection_yolo.cpp` was used to validate the YOLO v3 object detection algorithm in CPP through Opencv lib.
>- `Objectdection.cpp` and `Objectdection.h` were the object detection class which can be extended to find the targets in roboitc scenarios.
>- `Realsense.cpp` and `Realsense.h` were a cpp class, and can be used to realize some fundamental functions for the intel realsense 3D camera.
