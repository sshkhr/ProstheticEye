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

inline double L2norm(const pcl::PointXYZ &a,const pcl::PointXYZ &b){
	return sqrt(pow(a.x-b.x,2) + pow(a.y-b.y,2) + pow(a.z-b.z,2));
}

struct OFFData{
	std::vector<int> nodes;
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
	std::vector<Edge> edges;
	std::vector<double> weights;
	OFFData(std::vector<int> &_nodes, pcl::PointCloud<pcl::PointXYZ>::Ptr _source_cloud, std::vector<Edge> &_edges):
		nodes(_nodes),source_cloud(_source_cloud), edges(_edges){
		for(const auto &x:edges){
			weights.push_back(L2norm(source_cloud->points[x.first],source_cloud->points[x.second]));
		}
	}

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
			edges.push_back(Edge(i2,i1));
			added[i1][i2]=added[i2][i1]=true;
		}
		if(!(added[i3][i2]||added[i2][i3])){
			edges.push_back(Edge(i3,i2));
			edges.push_back(Edge(i2,i3));
			added[i3][i2]=added[i2][i3]=true;
		}
		if(!(added[i1][i3]||added[i3][i1])){
			edges.push_back(Edge(i1,i3));
			edges.push_back(Edge(i3,i1));
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

boost::shared_ptr<Visualizer> makeVisualizer(){
	boost::shared_ptr<Visualizer> viewer (new Visualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters ();
	return (viewer);
}

void findShortestPath(graph_t &g,vertex_descriptor s,vertex_descriptor t,std::vector<vertex_descriptor> &out){
	/*
	Graph must be connected
	Does not erase the output vector contents
	*/
  	property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);
  	std::vector<vertex_descriptor> p(num_vertices(g));
  	std::vector<int> d(num_vertices(g));

	dijkstra_shortest_paths(g, s,
		predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
		distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g))));

	do{
		out.push_back(t);
		if(t==p[t])
			return;
		t = p[t];
	}while(t!=s);
}

void pp_callback(const pcl::visualization::PointPickingEvent &e,void *in){
	uiState *state = (uiState *)in; 
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(state->patchCloud,
		std::get<0>(state->color),
		std::get<1>(state->color),
		std::get<2>(state->color));
	std::cout<<"Hello"<<std::endl;
	if(state->pointPickingDone){
		std::cout<<"Bye"<<std::endl;
		return;
	}
	if(!state->firstPointPicked){
		std::cout<<"Tarjan"<<std::endl;
		state->chosenPoints.clear();
		state->firstPointPicked = true;
		state->firstChosen = e.getPointIndex();
		state->lastChosen = e.getPointIndex();
		state->patchCloud->points.push_back(state->sourceCloud->points[e.getPointIndex()]);
		state->chosenPoints.push_back(e.getPointIndex());
	}
	else{
		int newPoint = e.getPointIndex();
  		findShortestPath(*state->graph,state->lastChosen,newPoint,state->chosenPoints);
  		for(const auto x:state->chosenPoints)
			state->patchCloud->points.push_back(state->sourceCloud->points[x]);	
  		state->lastChosen = newPoint;
	}
	state->visualizer->updatePointCloud(state->patchCloud,single_color,state->patchLabel);
}

void addCloud(boost::shared_ptr<Visualizer> viewer,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,const std::string &label){
	viewer->addPointCloud<pcl::PointXYZ> (cloud, label.c_str());
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, label.c_str());
}

void addRedCloud(boost::shared_ptr<Visualizer> viewer,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,const std::string &label){
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, label.c_str());
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, label.c_str());
}

int main(){
	OFFData d = parseOff("Full_face.off");
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (d.source_cloud);
	auto v = makeVisualizer();
	/*
	UI to select points
	Just keep shift + selecting, 
	u to undo,
	l to finish
	*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr patch(new pcl::PointCloud<pcl::PointXYZ>());
	const std::string patch_label("patch");
  	graph_t g(d.edges.begin(),d.edges.end(),d.weights.begin(),d.nodes.size());
	uiState state(v,d.source_cloud,patch,patch_label,&g);
  	v->registerPointPickingCallback(pp_callback,(void *)&state);
  	addCloud(v,d.source_cloud,"source");
  	addRedCloud(v,patch,patch_label);

	while(!v->wasStopped()){
		v->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}
	return 1;
}