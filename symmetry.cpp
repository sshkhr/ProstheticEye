#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <iostream>
#include <fstream>
#include <numeric>
#include <utility>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <vector> 

class symmetric_pair{
public:
	const pcl::PointXYZ &a;
	const pcl::PointXYZ &b;
	/*These are the co-ordinates of point on the plane closest to origin*/
	float alpha;
	float beta;
	float gamma;
	/*These are the spherical co-ordinates of point on the plane closest to origin*/
	//float theta;
	//float phi;
	//float r;
	symmetric_pair(pcl::PointXYZ &_a,pcl::PointXYZ &_b):
		a(_a),
		b(_b){
			const auto dz = a.z - b.z;
			const auto dy = a.y - b.y;
			const auto dx = a.x - b.x;
			const auto rho = a.x*a.x + a.y*a.y + a.z*a.z - b.x*b.x - b.y*b.y - b.z*b.z;
			const auto eta = 2*(dx*dx + dy*dy + dz*dz);
			alpha = dx*rho/eta;
			beta = dy*rho/eta;
			gamma = dz*rho/eta;
			//phi = atan2(gamma,sqrt(alpha*alpha + beta*beta));
			//theta = atan2(beta,alpha);
			//r = sqrt(alpha*alpha + beta*beta + gamma*gamma);
		}
};

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
	{
	  // --------------------------------------------------------
	  // -----Open 3D viewer and add point cloud and normals-----
	  // --------------------------------------------------------
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
return 
							-1*(
							(a.x*b.normal_x)+
							(a.y*b.normal_y)+
							(a.z*b.normal_z)
							)
							;
}



int main(){
	auto L2norm = [](pcl::PointXYZ a,pcl::Normal b){return sqrt(
							pow(a.x+b.normal_x,2) +
							pow(a.y+b.normal_y,2) +
							pow(a.z+b.normal_z,2)
								);
							};
	/*auto angleFind = [](pcl::PointXYZ a,pcl::Normal b){return 
							-1*(
							(a.x*b.normal_x)+
							(a.y*b.normal_y)+
							(a.z*b.normal_z)
							)
							;};
	std::cout<<acos(0)<<std::endl;
	getchar();
	*/const int sampling = 1000;
	const float pi = 3.1415;
	const float r = 1;
	srand(time(NULL));
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud2(new pcl::PointCloud<pcl::PointXYZ>());

    /*Generating 1000 points for 2 spheres randomly*/
	for(int i=0;i<sampling;i++){
		//float theta = (rand()%5000)/10.0*3.1415*2;
		float theta  = ((float)(rand()%100000))/100000*pi;
		float phi = ((float)(rand()%100000))/100000*2*pi;
		auto z = r*sin(phi);// + rand()%2/10.0;
		auto y = r*cos(phi)*sin(theta);// + rand()%2/10.0;
		auto x = r*cos(phi)*cos(theta);// + rand()%2/10.0;
		source_cloud->points.push_back(pcl::PointXYZ(x,y,z));
		source_cloud2->points.push_back(pcl::PointXYZ(x + 10,y,z));
	}

    /*Compute the normals*/
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation2;

	//for 1st point cloud
	normal_estimation.setInputCloud (source_cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);
	normal_estimation.setRadiusSearch (0.3);
	normal_estimation.compute (*cloud_with_normals);

	//for 2nd point cloud
	normal_estimation2.setInputCloud (source_cloud2);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation2.setSearchMethod (tree2);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals2 (new pcl::PointCloud<pcl::Normal>);
	normal_estimation2.setRadiusSearch (0.003);
	normal_estimation2.compute (*cloud_with_normals2);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;	
	viewer = normalsVis(source_cloud, cloud_with_normals);

	int k = 0;
	for(const auto x:cloud_with_normals2->points){
		if(std::isnan(x.normal_x)||std::isnan(x.normal_y)||std::isnan(x.normal_z)){
			k++;
		}
	}
	int g = 0;
	// int count = 0;
	// for(const auto x:cloud_with_normals->points){
	// 	if(std::isnan(x.normal_x)||std::isnan(x.normal_y)||std::isnan(x.normal_z)){
	// 		g++;
	// 	}
	// 	std::cout<<angleFind(source_cloud->points[count],x)<<"#"<<x<<"   "<<source_cloud->points[count++]<<std::endl; 
	// }
	float angle=0.0;
	int wrongthings = 0;
	for(int i=0;i<source_cloud->points.size();i++){
		auto x = cloud_with_normals->points[i];
		if(std::isnan(x.normal_x)||std::isnan(x.normal_y)||std::isnan(x.normal_z)){
			g++;
			continue;
		}
		angle = angleFind(source_cloud->points[i],x);
		if(angle<0.99) {
			wrongthings++;
			std::cout<<angle<<"#"<<x<<"   "<<source_cloud->points[i]<<std::endl; 
		}
		//std::cout<<angle<<"#"<<x<<"   "<<source_cloud->points[i]<<std::endl; 
	}
	std::cout<<"Nans:"<<k<<" "<<g<<std::endl;
	std::cout<<"Point pairs with angle jhamela: "<<wrongthings<<std::endl;
	  while (!viewer->wasStopped ())
	  {
	    viewer->spinOnce (100);
	    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	  }

	/*calculating principal curvatures and pruning point pairs if k1=k2 */
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation2;

	//for 1st point clo1ud
	principal_curvatures_estimation.setInputCloud (source_cloud);
	principal_curvatures_estimation.setInputNormals (cloud_with_normals);
	principal_curvatures_estimation.setSearchMethod (tree);
	principal_curvatures_estimation.setRadiusSearch (.01);
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
	principal_curvatures_estimation.compute (*principal_curvatures);

	//for 2nd point cloud
	principal_curvatures_estimation2.setInputCloud (source_cloud2);
	principal_curvatures_estimation2.setInputNormals (cloud_with_normals2);
	principal_curvatures_estimation2.setSearchMethod (tree2);
	principal_curvatures_estimation2.setRadiusSearch (.01);
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures2 (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
	principal_curvatures_estimation2.compute (*principal_curvatures2);

    /*store k1 and k2 values for both data sets in 4 arrays*/
	float k1a[principal_curvatures->points.size()],k2a[principal_curvatures->points.size()];
	float k1b[principal_curvatures2->points.size()],k2b[principal_curvatures2->points.size()];

	for(int i=1;i<principal_curvatures->points.size();i++){
		k1a[i]=principal_curvatures->points[i].pc1;
		k1b[i]=principal_curvatures2->points[i].pc1;
		k2a[i]=principal_curvatures->points[i].pc2;
		k2b[i]=principal_curvatures2->points[i].pc2;
	}

	std::cout << "output points.size (): " << principal_curvatures->points.size () << std::endl;
	std::cout << "output points.size (): " << principal_curvatures2->points.size () << std::endl;

	
	/*Decide points to ignore based on nearness of k values
    int ignore[principal_curvatures->points.size ()][principal_curvatures2->points.size ()]={0};
    for(int i=0;i<source_cloud->points.size();++i){
		for(int j=0;j<source_cloud2->points.size();++j){
			r=((k1a[i]-k1b[j])^2+(k2a[i]-k2b[j])^2);
			if (r>threshold)
				ignore[i][j]=1;
		}
	}


    /*creating symmetric pair for the plane from which the 2 points pass */
	std::vector<symmetric_pair> pairs;
	for(int i=0;i<source_cloud->points.size();++i){
		for(int j=0;j<source_cloud2->points.size();++j){
			pairs.push_back(
				symmetric_pair(
					source_cloud->points[i],
					source_cloud2->points[j]
				)
			);
		}
	}

	FILE *fpts1 = fopen("kvalues.txt","w+");
	for(int i=1;i<principal_curvatures->points.size();i++){
		fprintf(fpts1,"%f\t%f\t%f\t%f\n",k1a[i],k1b[i],k2a[i],k2b[i]);
		//std::cout<<k1a[i]<<k1b[i]<<k2a[i]<<k2b[i]<<std::endl;
	}
	fprintf(fpts1,"\n op size %ld",principal_curvatures->points.size ());
	fprintf(fpts1,"\n%f\n",principal_curvatures->points[0].pc1);
	fclose(fpts1);

	//std::cout<<pairs[0].a<<"->"<<pairs[0].b<<std::endl;
	//std::cout<<pairs[0].alpha<<" "<<pairs[0].beta<<" "<<pairs[0].gamma<<std::endl;

	FILE *fpts = fopen("centers.csv","w+");
	for(const auto pt:pairs){
		fprintf(fpts,"%f,%f,%f\n",pt.alpha,pt.beta,pt.gamma);
	}
	fclose(fpts);

    /*
	FILE *fout = fopen("all_pairs.csv","w+");
	for(const auto pt:pairs){
		fprintf(fout,"%f,%f,%f\n",pt.phi,pt.theta,pt.r);
	}
	fclose(fout);
	*/
}
