#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/principal_curvatures.h>

#include <iostream>
#include <fstream>
#include <numeric>
#include <utility>
#include <cmath>
#include <ctime>
#include <cstdlib>

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

int main(){
	const float pi = 3.1415;
	const float r = 1;
	srand(time(NULL));
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud2(new pcl::PointCloud<pcl::PointXYZ>());

    /*Generating 100 points for 2 spheres randomly*/
	for(int i=0;i<100;i++){
		float theta = (rand()%10)/10.0*3.1415;
		float phi = (rand()%10)/10.0*3.1415;
			auto z = r*sin(phi);// + rand()%2/10.0;
			auto y = r*cos(phi)*sin(theta);// + rand()%2/10.0;
			auto x = r*cos(phi)*cos(theta);// + rand()%2/10.0;
			source_cloud->points.push_back(pcl::PointXYZ(x,y,z));
			source_cloud2->points.push_back(pcl::PointXYZ(x + 10,y,z));
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

	std::cout<<pairs[0].a<<"->"<<pairs[0].b<<std::endl;
	std::cout<<pairs[0].alpha<<" "<<pairs[0].beta<<" "<<pairs[0].gamma<<" "<<pairs[0].theta<<" "<<pairs[0].phi<<std::endl;

	FILE *fpts = fopen("centers.csv","w+");
	for(const auto pt:pairs){
		fprintf(fpts,"%f,%f,%f\n",pt.alpha,pt.beta,pt.gamma);
	}
	fclose(fpts);


	FILE *fout = fopen("all_pairs.csv","w+");
	for(const auto pt:pairs){
		fprintf(fout,"%f,%f,%f\n",pt.phi,pt.theta,pt.r);
	}
	fclose(fout);
}
