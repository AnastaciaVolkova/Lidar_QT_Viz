# Lidar_QT_Viz
Project extends https://github.com/udacity/SFND_Lidar_Obstacle_Detection.
Demonstrates possibilities of PCL library with help of QT Gui.

Solved issues:
- wrong build of vtk - qt libraries not found - was not compiled for qt. Use ccmake
- share libraries directory should be included
- cache util of share libraries should be run
- pcl wasn't compiled for vtk-9, but for default vtk, which was got during installation - runtime errors
- pcl visualizer should be initialized with not default Renderer Window, but with vtkGeneriaOpenGLRenserWindow (another constructor)

