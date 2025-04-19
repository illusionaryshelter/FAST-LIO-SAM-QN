# FAST-LIO-SAM-QN

## Developing

+ Optional faster-lio features.

Thanks to the original author [engcang](https://github.com/engcang) for his help in the process!
+ This repository is a SLAM implementation combining [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) with pose graph optimization and loop closing based on [Quatro](https://quatro-plusplus.github.io/) and [Nano-GICP module](https://github.com/engcang/nano_gicp)
  + [Quatro](https://quatro-plusplus.github.io/) - fast, accurate and robust global registration which provides great initial guess of transform
  + [Quatro module](https://github.com/engcang/quatro) - `Quatro` as a module, can be easily used in other packages
  + [Nano-GICP module](https://github.com/engcang/nano_gicp) - fast ICP combining [FastGICP](https://github.com/SMRT-AIST/fast_gicp) + [NanoFLANN](https://github.com/jlblancoc/nanoflann)
+ Note: similar repositories already exist
  + [FAST_LIO_LC](https://github.com/yanliang-wang/FAST_LIO_LC): FAST-LIO2 + SC-A-LOAM based SLAM
  + [FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM): FAST-LIO2 + ScanContext based SLAM
  + [FAST_LIO_SAM](https://github.com/kahowang/FAST_LIO_SAM): FAST-LIO2 + LIO-SAM (not modularized)
  + [FAST_LIO_SAM](https://github.com/engcang/FAST-LIO-SAM): FAST-LIO2 + LIO-SAM (modularized)

## Video clip - https://youtu.be/MQ8XxRY472Y

## Dependencies

+ `C++` >= 17, `OpenMP` >= 4.5, `CMake` >= 3.10.0, `Eigen` >= 3.2, `Boost` >= 1.54

+ `ROS2`

+ [`GTSAM`](https://github.com/borglab/gtsam)

  ```shell
  wget -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip
  unzip gtsam.zip
  cd gtsam-4.1.1/
  mkdir build && cd build
  cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
  sudo make install -j16
  ```

+ [`Teaser++`](https://github.com/MIT-SPARK/TEASER-plusplus)

  ```shell
  git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
  cd TEASER-plusplus && mkdir build && cd build
  cmake .. -DENABLE_DIAGNOSTIC_PRINT=OFF
  sudo make install -j16
  sudo ldconfig
  ```

+ `tbb` (is used for faster `Quatro`)

  ```shell
  sudo apt install libtbb-dev
  ```

## How to build

+ Get the code and then build the main code.

  ```shell
  cd ~/your_workspace/src
  git clone https://github.com/illusionaryshelter/FAST-LIO-SAM-QN2.git --recursive
  cd ~/your_workspace
  colcon build 
  ```

## How to run

+ Then run (change config files in `FAST_LIO2`)

  ```shell
  ros2 launch fast_lio_sam_qn run.launch.py lidar:=ouster
  ros2 launch fast_lio_sam_qn run.launch.py lidar:=velodyne
  ros2 launch fast_lio_sam_qn run.launch.py lidar:=mid360
  ```

* In particular, we provide a preset launch option for specific datasets:

  - First you need to download [kitti2bag2](https://github.com/illusionaryshelter/kitti2bag2) and follow the README to convert the KITTI dataset to .bag file

  - Then run

    ```shell
    ros2 launch fast_lio_sam_qn run.launch.py lidar:=kitti
    (another ternimal)ros2 bag play yourbag
    ```

## Structure

+ odomPcdCallback
  + pub realtime pose in corrected frame
  + keyframe detection -> if keyframe, add to pose graph + save to keyframe queue
  + pose graph optimization with iSAM2
+ loopTimerFunc
  + process a saved keyframe
    + detect loop -> if loop, add to pose graph
+ visTimerFunc
  + visualize all **(Note: global map is only visualized once uncheck/check the mapped_pcd in rviz to save comp.)**

<br>

## Memo

+ `Quatro` module fixed for empty matches
+ `Quatro` module is updated with `optimizedMatching` which limits the number of correspondences and increased the speed

<br>

## License

<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a>

This work is licensed under a [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-nc-sa/4.0/)
