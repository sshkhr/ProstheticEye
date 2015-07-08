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



int main(){
	auto L2norm = [](pcl::PointXYZ a,pcl::Normal b){
		return sqrt(
				pow(a.x+b.normal_x,2) +
				pow(a.y+b.normal_y,2) +
				pow(a.z+b.normal_z,2)
				);
			};

	const int sampling = 1000;
	const float pi = 3.1415;
	const float ra = 1,rb = 0.5,rc = 1.5;
	srand(time(NULL));
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud2(new pcl::PointCloud<pcl::PointXYZ>());


	readfsamp("Scan2.xyz",source_cloud);

	std::cout<<"ReadSource"<<std::endl;

    /*Generating 1000 points for 2 spheres randomly
	for(int i=0;i<sampling;i++){
		float theta  = ((float)(rand()%100000))/100000*pi;
		float phi = ((float)(rand()%100000))/100000*2*pi;
		auto z = ra*sin(phi);// + rand()%2/10.0;
		auto y = rb*cos(phi)*sin(theta);// + rand()%2/10.0;
		auto x = rc*cos(phi)*cos(theta);// + rand()%2/10.0;
		source_cloud->points.push_back(pcl::PointXYZ(x,y,z));
		source_cloud->points.push_back(pcl::PointXYZ(x + 10,y,z));
	}*/

    /*Compute the normals*/
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;

	//for 1st point cloud
	normal_estimation.setInputCloud (source_cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);
	normal_estimation.setRadiusSearch (24);

	std::cout<<sizeof(symmetric_pair)<<std::endl;
	getchar();


	std::cout<<"\nComputingNormals"<<std::endl;

	normal_estimation.compute (*cloud_with_normals);

	std::cout<<"\nNormalsComputed"<<std::endl;

	int k = 0;
	for(const auto x:cloud_with_normals->points){
		if(std::isnan(x.normal_x)||std::isnan(x.normal_y)||std::isnan(x.normal_z)){
			k++;
		}
	}

	/*calculating principal curvatures and pruning point pairs if k1=k2*/ 
	pcl::PrincipalCurvaturesEstimation
		<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;

	//for 1st point cloud
	principal_curvatures_estimation.setInputCloud (source_cloud);
	principal_curvatures_estimation.setInputNormals (cloud_with_normals);
	principal_curvatures_estimation.setSearchMethod (tree);
	principal_curvatures_estimation.setRadiusSearch (.1);
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures 
		(new pcl::PointCloud<pcl::PrincipalCurvatures> ());

	std::cout<<"\nComputingPCs"<<std::endl;

	principal_curvatures_estimation.compute (*principal_curvatures);

	std::cout<<"\nComputingPCs"<<std::endl;


	auto rCd = [](pcl::PrincipalCurvatures a,pcl::PrincipalCurvatures b){
		return pow(a.pc1-b.pc1,2) + pow(a.pc2-b.pc2,2);};

	const float threshold=1e-5*0.7;
	std::vector<symmetric_pair> pairs;

    for(int i=0;i<source_cloud->points.size();i++){
		for(int j=0;j<source_cloud->points.size();j++){
			const float r = 
				rCd(principal_curvatures->points[i],principal_curvatures->points[j]);
			const bool print = (i==1 && ((rand()%10) == 0)) && false;//true;
			if(print){
				std::cout<<r<<"\t("
						 <<principal_curvatures->points[i].pc1<<","
						 <<principal_curvatures->points[i].pc2<<")\t("
						 <<principal_curvatures->points[j].pc1<<","
						 <<principal_curvatures->points[j].pc2<<")";
			}
			if(r<threshold){
				if(print)
					std::cout<<"!!!!";
				pairs.push_back(
					symmetric_pair(
						source_cloud->points[i],
						source_cloud->points[j],
						i,
						j
					)
				);
			}
			if(print)
				std::cout<<std::endl;
		}
	}


	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;	
	// viewer = normalsVis(source_cloud, cloud_with_normals);

	long int global_count = 0;
	for(int k = 0;k<source_cloud->points.size();k++){
		int count = 0;
		for(const auto x:pairs){
			if(x.i==k){
				count++;
				global_count++;
				// viewer->addLine<pcl::PointXYZ>(
				// 	source_cloud->points[x.i], 
				// 	source_cloud->points[x.j], 
				// 	boost::lexical_cast<std::string>(x.j).c_str()
				// );
			}
		}
		//std::cout<<k<<" "<<count<<std::endl;
	}
	std::cout<<global_count<<std::endl;
	std::cout<<global_count/((float)pow(2*sampling,2))<<std::endl;	

   //  while (!viewer->wasStopped ()){
	  //   viewer->spinOnce (100);
	  //   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	  // }

	// FILE *fpts = fopen("centers.csv","w+");
	// for(const auto pt:pairs){
	// 	fprintf(fpts,"%f,%f,%f\n",pt.alpha,pt.beta,pt.gamma);
	// }
	// fclose(fpts);

	FILE *fpts = fopen("data.ply","w+");

	fprintf(fpts,"ply\n");
	fprintf(fpts,"format ascii 1.0\n");
	fprintf(fpts,"element vertex %d\n",source_cloud->points.size());
	fprintf(fpts,"property float32 x\n");
	fprintf(fpts,"property float32 y\n");
	fprintf(fpts,"property float32 z\n"); 
	fprintf(fpts,"property float32 nx\n"); 
	fprintf(fpts,"property float32 ny\n"); 
	fprintf(fpts,"property float32 nz\n"); 
	fprintf(fpts,"end_header\n");
	for(int k = 0;k<source_cloud->points.size();k++){
		fprintf(fpts,"%f %f %f %f %f %f\n",
				source_cloud->points[k].x,source_cloud->points[k].y,source_cloud->points[k].z,
				cloud_with_normals->points[k].normal_x,cloud_with_normals->points[k].normal_y,cloud_with_normals->points[k].normal_z);
		}
	 fclose(fpts);
}
