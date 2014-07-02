#include<pcl-1.7/pcl/point_types.h>
#include<pcl-1.7/pcl/io/pcd_io.h>
#include<pcl-1.7/pcl/kdtree/kdtree.h>
#include<pcl-1.7/pcl/filters/extract_indices.h>
#include<pcl-1.7/pcl/segmentation/extract_clusters.h>
int main(int argc, char **argv)
{

pcl::PCDReader  reader;
pcl::PCDWriter writer;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
reader.read(argv[1], *cloud_ptr);

pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

std::vector<pcl::PointIndices> cluster_indices;

pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

ec.setClusterTolerance(0.02);
ec.setMinClusterSize(100);
ec.setMaxClusterSize(25000);

ec.setSearchMethod(tree);
ec.setInputCloud(cloud_ptr);
ec.extract(cluster_indices);
int j;
std::cout<<"number of cluster"<<cluster_indices.size();
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1(new pcl::PointCloud<pcl::PointXYZ>);

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cloudvector; 

std::vector<int> point_indices;

for(std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin();it!=cluster_indices.end();it++)
{

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      {cloud_cluster->points.push_back (cloud_ptr->points[*pit]); //*

point_indices.push_back(*pit);
             }

 pcl::copyPointCloud(*cloud_ptr,point_indices,*cloud_cluster1);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    cloudvector.push_back(cloud_cluster);

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;

}


}
