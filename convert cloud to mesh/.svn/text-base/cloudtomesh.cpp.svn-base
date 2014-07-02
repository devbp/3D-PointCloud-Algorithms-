
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.7/pcl/features/normal_3d.h>
#include <pcl-1.7/pcl/surface/gp3.h>
#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include<pcl-1.7/pcl/surface/mls.h>
#include<ctime>

//#include<boost/timer.hpp>
#include <boost/progress.hpp>
//#include <pcl-1.7/pcl/surface/vtk_smoother.h>
int main (int argc, char** argv)
{
//boost:: progress_timer t; 
clock_t tStart = clock();

boost::timer t;

//printf("Execution Time: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

 // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile (argv[1], *cloud);
  //pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud

pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (*mls_points);



/*
  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (500);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);
/*
pcl::surface::VTKSmoother vtkSmoother;

vtkSmoother.convertToVTK(triangles);

vtkSmoother.smoothMeshWindowedSinc();

vtkSmoother.convertToPCL(triangles);

*/


printf("Execution Time: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
std::cout<<"elapsed time"<<t.elapsed();

std::string s="normal";
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new    pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
   
  viewer->addPointCloudNormals<pcl::PointNormal>(mls_points,10,0.5,"normals");
// viewer->addPolygonMesh(triangles,"cloud mesh"); 
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.2, "cloud mesh");
  
  viewer->addCoordinateSystem (0.0);
  viewer->initCameraParameters ();
   while (!viewer->wasStopped ())
    {
    viewer->spinOnce (100);
   
    }





  // Additional vertex information
  //std::vector<int> parts = gp3.getPartIDs();
  //std::vector<int> states = gp3.getPointStates();

  // Finish
  return (0);
}


