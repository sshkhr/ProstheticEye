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
		if(rand()%50==0)
    		cld->points.push_back(pcl::PointXYZ(a,b,c));
	}
}

class symmetric_pair{
public:
	const pcl::PointXYZ &a;
	const pcl::PointXYZ &b;
	const unsigned int i;
	const unsigned int j;
	/*These are the co-ordinates of point on the plane closest to origin*/
	float alpha;
	float beta;
	float gamma;
	symmetric_pair(pcl::PointXYZ &_a,pcl::PointXYZ &_b,unsigned int _i,unsigned int _j):
		a(_a),
		b(_b),
		i(_i),
		j(_j){
			const auto dz = a.z - b.z;
			const auto dy = a.y - b.y;
			const auto dx = a.x - b.x;
			const auto rho = a.x*a.x + a.y*a.y + a.z*a.z - b.x*b.x - b.y*b.y - b.z*b.z;
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
