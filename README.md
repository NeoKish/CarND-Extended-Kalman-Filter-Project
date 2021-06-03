# Extended Kalman Filter Project 


This project is part of Udacity's Self Driving Car NanoDegree which involves utilizing a kalman filter(for Lidar which has linear measurements) and extended Kalman filter(for Radar which gives out non-linear measurements) to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining Root Mean Square Error values that are lower than the tolerance outlined in the project rubric. 

The project is run on C++ script and the results are visualized on simulator provided by Udacity
![image](https://user-images.githubusercontent.com/66986430/120668677-4c646900-c4ac-11eb-9ecd-f4280e32198c.png)

The project was provided with a starter code. As requirement of the project program code for src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp and tools.cpp had to be completed with initialization of state and covariance matrices, prediction and update step with update of measurements from Lidar and Radar.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `






