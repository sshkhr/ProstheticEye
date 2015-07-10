
#include "symmetry.hpp"

int main(){
	auto L2norm = [](pcl::PointXYZ a,pcl::Normal b){
		return sqrt(
				pow(a.x+b.normal_x,2) +
				pow(a.y+b.normal_y,2) +
				pow(a.z+b.normal_z,2)
				);
			};

	//const int sampling = 1000;
	//const float pi = 3.1415;
	//const float ra = 1,rb = 0.5,rc = 1.5;
	srand(time(NULL));
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	readfsamp("Full_face.xyz",source_cloud);

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

	for(const auto x:source_cloud->points)
		std::cout<<x<<std::endl;
	getchar();

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
	principal_curvatures_estimation.setRadiusSearch (20);
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures 
		(new pcl::PointCloud<pcl::PrincipalCurvatures> ());

	std::cout<<"\nComputingPCs"<<std::endl;

	principal_curvatures_estimation.compute (*principal_curvatures);

	std::cout<<"\nComputingPCs"<<std::endl;


	// auto rCd = [](pcl::PrincipalCurvatures a,pcl::PrincipalCurvatures b){
	// 	return pow(a.pc1-b.pc1,2) + pow(a.pc2-b.pc2,2);};

	const float threshold=1e-5*0.7;
	std::vector<symmetric_pair> pairs;

	pcl::PointCloud<pcl::PointXYZ>::Ptr signature_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(const auto x: principal_curvatures->points){
		signature_cloud->points.push_back(pcl::PointXYZ(x.pc1,x.pc2,0));
	}

	std::cout<<"Testing kdtree"<<std::endl;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (signature_cloud);
	//pcl::PointXYZ searchPoint(5,5,0);


	int K = 11;
	std::vector<int> pointIdxNKNSearch(K);
 	std::vector<float> pointNKNSquaredDistance(K);
 	//MIGHT NEED to use random drop while choosing pairs.
  	for(int i = 0; i< signature_cloud->points.size();i++){
  		if(!pointIsValid(signature_cloud->points[i])){
  			continue;
  		}
  		std::cout<<signature_cloud->points[i];
  		if ( kdtree.nearestKSearch (signature_cloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
    		for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j){
      			if(pointNKNSquaredDistance[j]<threshold){
      				pairs.push_back(symmetric_pair(source_cloud->points[i],source_cloud->points[pointIdxNKNSearch[j]],i,j));
      			}
      		}
      		std::cout<<"  Nearest - "<<signature_cloud->points[pointIdxNKNSearch[1]]<<std::endl;
  		}
  	}

  	//printing transformation space to file
  	FILE *fpts = fopen("transformspace.csv","w+");
	for(const auto pt:pairs){
		if(!pairIsValid(pt)){
			continue;
		}
		fprintf(fpts,"%f,%f,%f\n",pt.alpha,pt.beta,pt.gamma);
	}
	fclose(fpts);

  	

 //    for(int i=0;i<source_cloud->points.size();i++){
	// 	for(int j=0;j<source_cloud->points.size();j++){
	// 		const float r = 
	// 			rCd(principal_curvatures->points[i],principal_curvatures->points[j]);
	// 		const bool print = (i==1 && ((rand()%10) == 0)) && false;//true;

	// 		if(print){
	// 			std::cout<<r<<"\t("
	// 					 <<principal_curvatures->points[i].pc1<<","
	// 					 <<principal_curvatures->points[i].pc2<<")\t("
	// 					 <<principal_curvatures->points[j].pc1<<","
	// 					 <<principal_curvatures->points[j].pc2<<")";
	// 		}
	// 		if(r<threshold){
	// 			if(print)
	// 				std::cout<<"!!!!";
	// 			pairs.push_back(
	// 				symmetric_pair(
	// 					source_cloud->points[i],
	// 					source_cloud->points[j],
	// 					i,
	// 					j
	// 				)
	// 			);
	// 		}
	// 		if(print)
	// 			std::cout<<std::endl;
	// 	}
	// }

	// pcl::PointCloud<pcl::PointXYZ>::Ptr compare_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// for(const auto pair:pairs){
	// 	if (pair.i==100){
	// 		compare_cloud->points.push_back(pair.b);
	// 	}
	// 	if(pair.j==100){
	// 		compare_cloud->points.push_back(pair.a);
	// 	}
	// }
    
	// long int global_count = 0;
	// for(int k = 0;k<source_cloud->points.size();k++){
	// 	int count = 0;
	// 	for(const auto x:pairs){
	// 		if(x.i==k){
	// 			count++;
	// 			global_count++;
	// 			// viewer->addLine<pcl::PointXYZ>(
	// 			// 	source_cloud->points[x.i], 
	// 			// 	source_cloud->points[x.j], 
	// 			// 	boost::lexical_cast<std::string>(x.j).c_str()
	// 			// );
	// 		}
	// 	}
	// 	//std::cout<<k<<" "<<count<<std::endl;
	// }
	// std::cout<<global_count<<std::endl;
	// std::cout<<global_count/((float)pow(2*source_cloud->points.size(),2))<<std::endl;	

	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;	
	// viewer = normalsVis(source_cloud, cloud_with_normals);

 //    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(compare_cloud, 255, 0, 0);
 //    viewer->addPointCloud<pcl::PointXYZ> (compare_cloud, single_color, "compare cloud");

 //    while (!viewer->wasStopped ()){
	//      viewer->spinOnce (100);
	//   	   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	//    }

	// FILE *fpts = fopen("centers.csv","w+");
	// for(const auto pt:pairs){
	// 	fprintf(fpts,"%f,%f,%f\n",pt.alpha,pt.beta,pt.gamma);
	// }
	// fclose(fpts);

	// FILE *fpts = fopen("data.ply","w+");

	// fprintf(fpts,"ply\n");
	// fprintf(fpts,"format ascii 1.0\n");
	// fprintf(fpts,"element vertex %lu\n",source_cloud->points.size());
	// fprintf(fpts,"property float32 x\n");
	// fprintf(fpts,"property float32 y\n");
	// fprintf(fpts,"property float32 z\n"); 
	// fprintf(fpts,"property float32 nx\n"); 
	// fprintf(fpts,"property float32 ny\n"); 
	// fprintf(fpts,"property float32 nz\n"); 
	// fprintf(fpts,"end_header\n");
	// for(int k = 0;k<source_cloud->points.size();k++){
	// 	fprintf(fpts,"%f %f %f %f %f %f\n",
	// 			source_cloud->points[k].x,source_cloud->points[k].y,source_cloud->points[k].z,
	// 			cloud_with_normals->points[k].normal_x,cloud_with_normals->points[k].normal_y,cloud_with_normals->points[k].normal_z);
	// 	}
	//  fclose(fpts);


}
