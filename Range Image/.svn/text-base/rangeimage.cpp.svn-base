#include <pcl-1.7/pcl/range_image/range_image.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>

#include <pcl-1.7/pcl/visualization/range_image_visualizer.h>

int main (int argc, char** argv) {
 
pcl::PointCloud<pcl::PointXYZ> pointCloud;
 pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], pointCloud);

  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
 float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.000;
  float minRange = 0.0f;
  int borderSize = 5;
  
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
  std::cout << rangeImage << "created range image\n";

pcl::visualization::RangeImageVisualizer* range_viewer = NULL;
  range_viewer =
    pcl::visualization::RangeImageVisualizer::getRangeImageWidget (rangeImage, -std::numeric_limits<float>::infinity (), std::numeric_limits<float>::infinity (), false, "Range image with borders");
while(1)
range_viewer->spinOnce ();
/*
while (!range_viewer->wasStopped ())
  {
    range_viewer->spinOnce ();
    range_viewer.spinOnce ();
    pcl_sleep(0.01);
  }
*/

}


