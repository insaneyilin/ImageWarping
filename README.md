# ImageWarping

C++ Implementation of some Image warping algorithms.

## Dependencies

- Qt 5
- OpenCV
- CMake
- Eigen

## Build

Please change [`CMAKE_PREFIX_PATH`](https://github.com/insaneyilin/ImageWarping/blob/master/CMakeLists.txt#L8) and [`EIGEN_ROOT_DIR`](https://github.com/insaneyilin/ImageWarping/blob/master/CMakeLists.txt#L14) in `CMakeLists.txt` according to your environment.

```
mkdir build
cd build
cmake ..
make
```

## Screenshots

| ![Original](https://github.com/insaneyilin/ImageWarping/blob/master/screenshots/screenshots_0_origin.png)  | ![Select control points](https://github.com/insaneyilin/ImageWarping/blob/master/screenshots/screenshots_0_select_ctrl_points.png) | ![Warped](https://github.com/insaneyilin/ImageWarping/blob/master/screenshots/screenshots_0_warped.png) |
|:---:|:---:|:---:|
| Original Image | Control points | Warped Image | 

## Todo

- [x] Implement Radial Basis Function warping
- [ ] Inverse Distance Weighted warping
