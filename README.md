# Unscented Kalman Filter
This repository presents an unscented kalman filter (UKF) library and one of its applications as a sample use case. 

# Sample use case: Lidar and radar sensor fusion

First, check out readme from [here](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project)

Sensor fusion algorithm based on UKF is used to track a vehicle. It predicts 2D coordinate and velocity of the vehicle from lidar and radar measurements. Results are shown on the animation below. You can check out full video from [here](https://youtu.be/cj7iHbJsKlU)

![Sensor fusion](https://media.giphy.com/media/l0HU41mMmatauiJoI/giphy.gif "Sensor Fusion")

Red and blue circles correspond to lidar and radar measurements, respectively. Green triangles are the predicted coordinates of the vehicle. My px, py, vx, and vy RMSE are less than to the values [.09, .10, .40, .30].

# Quickstart 
## Unscented Kalman Filter
You can use unscented kalman filter library presented here in your own projects. You'll only need source files under UKF folder.

```bash
UKF/kalman_filter.h
UKF/kalman_filter.cpp
UKF/unscented_kalman_filter.h
UKF/unscented_kalman_filter.cpp
```

Additionally, you'll need to provide implementations of certain functions which are defined as pure virtual in `unscented_kalman_filter.h`. Please feel free to inspect `ukf_lidar.h` or `ukf_radar.h` files to see how to inherit from `UnscentedKalmanFilter` class and implement those necessary functions.

## Sensor fusion
If you want to use UKF for sensor fusion, the most simple and the cleanest way is to initialize seperate filters for each sensor type and keep them synced to each other when any of the filters gets an update. Please feel free to inspect `ukf_fusion.h` to understand fusion pipeline.

