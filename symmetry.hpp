#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
	
#include <boost/lexical_cast.hpp>


#include <iostream>
#include <fstream>
#include <numeric>
#include <utility>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <vector> 

void readf(const std::string &x ,pcl::PointCloud<pcl::PointXYZ>::Ptr cld)
{
	std::ifstream infile(x);
	float a, b, c;
	while (infile >> a >> b >> c)
	{
      cld->points.push_back(pcl::PointXYZ(a,b,c));
	}
}

void readfsamp(const std::string &x ,pcl::PointCloud<pcl::PointXYZ>::Ptr cld)
{
	std::ifstream infile(x);
	float a, b, c;

	while (infile >> a >> b >> c)
	{
		if(rand()%1==0) //change to rand()%50 when generating the transform space for Vinayak face: Full_face.xyz
    		cld->points.push_back(pcl::PointXYZ(a,b,c));
	}
}

class symmetric_pair{
public:
	const unsigned int i;
	const unsigned int j;
	/*These are the co-ordinates of point on the plane closest to origin*/
	float alpha;
	float beta;
	float gamma;
	symmetric_pair(pcl::PointXYZ &_a,pcl::PointXYZ &_b,unsigned int _i,unsigned int _j):
		i(_i),	
		j(_j){
			const auto dz = _a.z - _b.z;
			const auto dy = _a.y - _b.y;
			const auto dx = _a.x - _b.x;
			const auto rho = _a.x*_a.x + _a.y*_a.y + _a.z*_a.z - _b.x*_b.x - _b.y*_b.y - _b.z*_b.z;
			const auto eta = 2*(dx*dx + dy*dy + dz*dz);
			alpha = dx*rho/eta;
			beta = dy*rho/eta;
			gamma = dz*rho/eta;
		}
};

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals){
	  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  viewer->setBackgroundColor (0, 0, 0);
	  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
	  viewer->initCameraParameters ();
	  return (viewer);
	}

float angleFind(pcl::PointXYZ a,pcl::Normal b){
//std::cout<<a<<"  &&& "<<b<<std::endl;
	return -1*((a.x*b.normal_x)+(a.y*b.normal_y)+(a.z*b.normal_z));
}

bool normalIsValid(pcl::Normal &x){
	if(std::isnan(x.normal_x)||std::isnan(x.normal_y)||std::isnan(x.normal_z)){
			return true;
	}
	return false;
}

bool pointIsValid(pcl::PointXYZ &x){
	if(std::isnan(x.x)||std::isnan(x.y)||std::isnan(x.z)){
			return false;
	}
	return true;
}

bool pairIsValid(const symmetric_pair &x){
	if(std::isnan(x.alpha)||std::isnan(x.beta)||std::isnan(x.gamma)){
			return false;
	}
	return true;
}
