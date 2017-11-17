#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath> 
#include <cstdlib>
#include <time.h>
#include <string> 
#include <ctime>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/common/transforms.h>
#include <pcl/octree/octree.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

#include <pcl/point_types.h>
#include <boost/lexical_cast.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr save_pc (new pcl::PointCloud<pcl::PointXYZ> ());
// Function to acquire point cloud using Kinect Camera
class SimpleOpenNIViewer
 {
   public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}
         

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
     {
       if (!viewer.wasStopped())
        {
          viewer.showCloud (cloud);
          copyPointCloud(*cloud, *save_pc);

        }
     }
     
     void run ()
     {
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

      boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

      interface->registerCallback (f);

      interface->start ();

      while (!viewer.wasStopped())
         boost::this_thread::sleep (boost::posix_time::seconds (1));
       
      interface->stop ();

      pcl::io::savePCDFileASCII("test.pcd", *save_pc );
      std::cout<<"Saved"<<std::endl;
     }



     pcl::visualization::CloudViewer viewer;
 };

int main ()
{
	  /* --------------------------------------------------------------------------------------
	  // To Capture New Point Cloud
	  // -------------------------------------------------------------------------------------- */

	  SimpleOpenNIViewer v;
      v.run ();
	  

}