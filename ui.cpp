#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

#include <iostream>
#include <fstream>
#include <numeric>
#include <utility>
#include <ctime>
#include <cstdlib>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Epick;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned,Epick> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
typedef CGAL::Delaunay_triangulation_2<Epick,Tds> Delaunay_Mesh;
typedef Epick::Point_2 Point_2;
typedef Epick::Point_3 Point_3;

using namespace boost;

typedef adjacency_list < listS, vecS, directedS,
  no_property, property < edge_weight_t, double > > graph_t;
typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
typedef std::pair<int, int> Edge;

typedef pcl::visualization::PCLVisualizer Visualizer;
struct OFFData{
	std::vector<int> nodes;
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
	std::vector<Edge> edges;
	OFFData(std::vector<int> &_nodes, pcl::PointCloud<pcl::PointXYZ>::Ptr _source_cloud, std::vector<Edge> &_edges):
		nodes(_nodes),source_cloud(_source_cloud), edges(_edges){}

};

struct uiState{
	boost::shared_ptr<Visualizer> visualizer;
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr patchCloud;
	std::string patchLabel;	
	graph_t *graph;
	std::vector<vertex_descriptor> chosenPoints;
	int firstChosen;
	int lastChosen;
	bool pointPickingDone;
	bool firstPointPicked;
	bool WASDenabled;
	std::tuple<double,double,double> color;

	uiState(boost::shared_ptr<Visualizer> _v,
				pcl::PointCloud<pcl::PointXYZ>::ConstPtr _s,
				pcl::PointCloud<pcl::PointXYZ>::Ptr _p,
				std::string _l,
				graph_t *_g){
		visualizer = _v;
		sourceCloud = _s;
		patchCloud = _p;
		patchLabel = _l;
		graph = _g;
		color = std::make_tuple(255,0,0);
		this->reset();
	}

	void reset(){
		firstChosen = 0;
		lastChosen = 0;
		pointPickingDone = false;
		firstPointPicked = false;
		chosenPoints.clear();
		WASDenabled = false;
	}
};

inline double L2norm(const pcl::PointXYZ &a,const pcl::PointXYZ &b){
	return sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2) + pow(a.z-b.z,2));
}

OFFData parseOff(const std::string &filename){
	std::ifstream infile(filename);
	std::string s;
	infile>>s;
	int length, faces, e;
	infile>>length>>faces>>e;
	//Nodes
	std::vector<int> nodes(length);
	for(int i =0;i<length;i++){
		nodes[i] = i;
	}
	//Point Cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	float x, y, z;
	for(int i = 0; i<length;i++){
		infile >> x >> y >> z;
		//std::cout<<x<<" "<<y<<" "<<z<<std::endl;
		source_cloud->points.push_back(pcl::PointXYZ(x,y,z));
	}
	std::cout<<"Done"<<std::endl;
	//Edges
	//bool added[length][length];
	std::vector<std::vector<bool> >added(length);
	// for(auto x:added)
	// 	x.resize(length);
	puts("Done");
	for(int i = 0;i<length;i++){
		for(int j = 0; j<length;j++){
			//std::cout<<i<<" "<<j<<std::endl;
			added[i].push_back(false);
		}
	}
	std::vector<Edge> edges;
	int i0, i1, i2, i3;
	for(int i = 0; i<faces;i++){
		infile>>i0>>i1>>i2>>i3;
		if(!(added[i1][i2]||added[i2][i1])){
			edges.push_back(Edge(i1,i2));
			added[i1][i2]=added[i2][i1]=true;
		}
		if(!(added[i3][i2]||added[i2][i3])){
			edges.push_back(Edge(i3,i2));
			added[i3][i2]=added[i2][i3]=true;
		}
		if(!(added[i1][i3]||added[i3][i1])){
			edges.push_back(Edge(i1,i3));
			added[i1][i3]=added[i3][i1]=true;
		}
	}
	infile.close();
	//returning the data
	return OFFData(nodes,source_cloud,edges);

}

inline bool pointIsValid(pcl::PointXYZ &x){
	if(std::isnan(x.x)||std::isnan(x.y)||std::isnan(x.z)){
			return false;
	}
	return true;
}

inline float getNorm(pcl::PointXYZ point){
	return pow(point.x,2)+pow(point.y,2)+pow(point.z,2);
}

inline float reflectionCoeff(pcl::PointXYZ plane, pcl::PointXYZ point){
	const float d = getNorm(plane); // check if passing *& blah
	return 2*(d-(point.x*plane.x)-(point.y*plane.y)-(point.z*plane.z))/d;
}

pcl::PointXYZ findReflection(pcl::PointXYZ plane, pcl::PointXYZ point){
	const float coeff = reflectionCoeff(plane, point);
	return pcl::PointXYZ(point.x+(plane.x*coeff),point.y+(plane.y*coeff),point.z+(plane.z*coeff));
}

pcl::PointXYZ findClosestReflection(pcl::PointXYZ plane, pcl::PointXYZ point, 
								pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){
	//Reflect the point
	//float coeff = reflectionCoeff(plane, point);
	pcl::PointXYZ reflected_point = findReflection(plane, point);//(point.x+(plane.x*coeff),point.y+(plane.y*coeff),point.z+(plane.z*coeff));
	
	//Find nearest neighbor
	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);
 	std::vector<float> pointNKNSquaredDistance(K);
 // 	if(!pointIsValid(reflected_point)){
	// 	return NULL;
	// }
	//std::cout<<signature_cloud->points[i];
	if ( kdtree.nearestKSearch (reflected_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
		return source_cloud->points[pointIdxNKNSearch[0]];
	}
	return pcl::PointXYZ(0,0,0);
}

int main(){
	OFFData d = parseOff("Full_face.off");
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (d.source_cloud);
	//pcl::PointXYZ test_point(0,0,0);
	//std::cout<<"For point "<<test_point<<", reflection is on: ";
	//std::cout<<findReflection(pcl::PointXYZ(1,1,1),test_point)<<std::endl;
	
	//std::cout<<findClosestReflection(pcl::PointXYZ(9.327223,0.468285,0.717058),d.source_cloud->points[2000],kdtree,d.source_cloud)<<std::endl;
	
	// for(int i=0;i<10;i++){
	// 	std::cout<<d.nodes[i]<<" : "<<d.source_cloud->points[i]<<std::endl;
	// }
	// for(int i=0;i<10;i++){
	// 	std::cout<<d.nodes[i]<<" : "<<d.edges[i].first<<" "<<d.edges[i].second<<std::endl;
	// }

	return 0;
}