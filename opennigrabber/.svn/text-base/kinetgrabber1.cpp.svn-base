#include <iostream>
#include <string>
#include <sstream>

#include <pcl-1.7/pcl/point_cloud.h>

#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/io/openni_grabber.h>

#include <pcl-1.7/pcl/visualization/cloud_viewer.h>

using namespace std;

//const string OUT_DIR = "D:\\frame_saver_output\\";

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer ("PCL Viewer")
    {
                frames_saved = 0;
                save_one = false;
    }
      
 void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
    
//void cloud_cb_ (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> > &cloud ) 
   // void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
                if (!viewer.wasStopped()) {
                        viewer.showCloud (cloud);
    
                          // std::cout<<"number of points"<<cloud->points.size()<<std::endl;
           
                                std::stringstream out;
                                out << frames_saved;
                                std::string name = "cloud" + out.str() + ".pcd";
                                pcl::io::savePCDFileASCII( name, *cloud );
                                
                        
                }

 
    }

    void run ()
    {
                pcl::OpenNIGrabber* interface = new pcl::OpenNIGrabber();
 
            //boost::function<void (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&) > f =
              //            boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

                     boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

                    //boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

                boost::signals2::connection sg=  interface->registerCallback (f);

                interface->start ();
       
                char c;

                while (!viewer.wasStopped())
                {
                      boost::this_thread::sleep (boost::posix_time::seconds (1));

                            cout << "Saving frame " << frames_saved << ".\n";
                                frames_saved++;
           
                }

                //std::cout<<"Frames rate per second"<<interface->getFramesPerSecond()<<std::endl;
                interface->stop ();
        }

        
   
        private:
                int frames_saved;
                bool save_one;
      pcl::visualization::CloudViewer viewer;

};

int main ()
{
    SimpleOpenNIViewer v;
    v.run ();
    return 0;
} 
