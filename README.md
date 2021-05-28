# Lidar_QT_Viz
Project extends [Lidar Obstacle detection by Udacity _Sensor Fusion Nanodegree Program_](https://github.com/udacity/SFND_Lidar_Obstacle_Detection) and demonstrates possibilities of PCL library with help of QT Gui.

## What is done
* Real point cloud is filtered, segmented and clustered
* Obstacles in a driving environment are detected

## New feautures
* Use QT library (VTK+PCL+QT)
* Use GUI to switch obstacle detection stages
* Use GUI to adjust filter, segmentation and clustering parameters

## Solved issues:
- wrong build of vtk - qt libraries not found - was not compiled for qt. Use ccmake
- share libraries directory should be included
- cache util of share libraries should be run
- pcl wasn't compiled for vtk-9, but for default vtk, which was got during installation - runtime errors
- pcl master was used 26-05-2021 18:22:47. It was built with boost_1_65
- pcl package was not used as it refers to vtk-8
- pcl visualizer should be initialized with not default Renderer Window, but with vtkGenericOpenGLRenderWindow (another constructor)
- header _QVTKRenderWidget.h_ from VTK source directory was copied manually to /usr/local/include

