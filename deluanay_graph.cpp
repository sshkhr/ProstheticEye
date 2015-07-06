#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>

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

boost::shared_ptr<Visualizer> makeVisualizer(){
	boost::shared_ptr<Visualizer> viewer (new Visualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters ();
	return (viewer);
}

void addRGBCloud(boost::shared_ptr<Visualizer> viewer,pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,const std::string &label){
  	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, label.c_str());
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, label.c_str());
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

void addYellowCloud(boost::shared_ptr<Visualizer> viewer,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,const std::string &label){
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, label.c_str());
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, label.c_str());
}


void kb_callback(const pcl::visualization::KeyboardEvent &e,void *in){
	uiState *state = (uiState *)in;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(state->patchCloud,
		std::get<0>(state->color),
		std::get<1>(state->color),
		std::get<2>(state->color));	
	if(!state->firstPointPicked)
		return;
	if(!state->pointPickingDone){
		if((e.getKeySym()=="l"||e.getKeySym()=="L")&&e.keyDown()){
			state->chosenPoints.clear();
			findShortestPath(*state->graph,
				state->lastChosen,state->firstChosen,state->chosenPoints);
	  		for(const auto x:state->chosenPoints)
				state->patchCloud->points.push_back(state->sourceCloud->points[x]);
			state->visualizer->updatePointCloud(state->patchCloud,single_color,state->patchLabel);
			state->pointPickingDone = true;
		}
	}
	else if(state->WASDenabled && e.keyDown()){
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(state->patchCloud,
			std::get<0>(state->color),
			std::get<1>(state->color),
			std::get<2>(state->color));
		switch(e.getKeySym()[0]){
			case 'w':
			case 'W':
				for(auto &pt:state->patchCloud->points){
					pt.x += 1;
				}
				break;
			case 'a':
			case 'A':
				for(auto &pt:state->patchCloud->points){
					pt.y += 1;
				}
				break;
			case 'd':
			case 'D':
				for(auto &pt:state->patchCloud->points){
					pt.y -= 1;
				}
				break;
			case 's':
			case 'S':
				for(auto &pt:state->patchCloud->points){
					pt.x -= 1;
				}
				break;
			case 'z':
			case 'Z':
				for(auto &pt:state->patchCloud->points){
					pt.z += 1;
				}
				break;
			case 'x':
			case 'X':
				for(auto &pt:state->patchCloud->points){
					pt.z -= 1;
				}
				break;
			default:;
		};
		state->visualizer->updatePointCloud(state->patchCloud,single_color,state->patchLabel);
	}
}

void pp_callback(const pcl::visualization::PointPickingEvent &e,void *in){
	uiState *state = (uiState *)in; 
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(state->patchCloud,
		std::get<0>(state->color),
		std::get<1>(state->color),
		std::get<2>(state->color));
	if(state->pointPickingDone)
		return;
	std::cout<<e.getPointIndex()<<std::endl;
	if(!state->firstPointPicked){
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

int main(){
	srand(time(NULL));

	/*
	Create random points in XY plane and insert into a delaunay triangulation
	Assign indexes to the nodes of the triangulation
	*/
	const int count = 10000;
	//auto Z = [](double x,double y){return (x*x + y*y)/200;};
	auto Z = [](double x,double y){return (x*x*x/3.0 - 625*x)/500;};

	Delaunay_Mesh dt;
	for(int i=0;i<count;i++){
		auto x = rand()%100-50;
		auto y = rand()%100-50;
		auto vh = dt.insert(Point_2(x,y));
		vh->info() = i;
	}

	/*
	Copy indexwise into STL vector and pcl PointCloud
	*/

	std::vector<Point_2> points_2(count);
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_3(new pcl::PointCloud<pcl::PointXYZ>());
	points_3->points.resize(count);
	for(auto vit = dt.finite_vertices_begin(); vit!=dt.finite_vertices_end(); ++vit){
		points_2[vit->info()] = vit->point();
		points_3->points[vit->info()] = pcl::PointXYZ(vit->point().x(),
				vit->point().y(),Z(vit->point().x(),vit->point().y())); 
	}

	/*
	Build boost graph
	Node list consists of indexes in dt
	Edge list is built as a bidirectional graph with adjacency information
	from CGAL circulators
	*/

	std::vector<int> nodes(count);
	std::iota(nodes.begin(),nodes.end(),0);

	std::vector<Edge> edges;
	std::vector<double> weights;
	for(auto vit = dt.finite_vertices_begin();vit!=dt.finite_vertices_end();++vit){
		auto circ = dt.incident_vertices(vit);
		auto begin = circ;
		do{
			if(!dt.is_infinite(circ)){
				edges.push_back(Edge(vit->info(),circ->info()));
				weights.push_back(L2norm(points_3->points[vit->info()],points_3->points[circ->info()]));
			}
		}while(++circ!=begin);
	}

  	graph_t g(edges.begin(), edges.end(), weights.begin(), nodes.size());

  	/*
	Start a new window and begin visualizing the point cloud
	Add the source cloud and attach callbacks for point picking
	When point is picked, search for the shortest path to the previously
		selected point (if any)
	Add these points + the path as a new point cloud which is colored red
  	*/

  	pcl::PointCloud<pcl::PointXYZ>::Ptr upper_patch(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr lower_patch(new pcl::PointCloud<pcl::PointXYZ>());
    const std::string source_label("source");
    const std::string upper_label("upper");
    const std::string lower_label("lower");
  	auto v = makeVisualizer();
  	addCloud(v,points_3,source_label);
  	addRedCloud(v,upper_patch,upper_label);
  	addYellowCloud(v,lower_patch,lower_label);

	uiState state(v,points_3,upper_patch,upper_label,&g);
  	v->registerPointPickingCallback(pp_callback,(void *)&state);
  	v->registerKeyboardCallback(kb_callback,(void *)&state);

	while(!v->wasStopped()&&!state.pointPickingDone){
		v->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}

	state.reset();
	state.patchCloud = lower_patch;
	state.patchLabel = lower_label;
	state.color = std::make_tuple(255,255,0);

	while(!v->wasStopped()&&!state.pointPickingDone){
		v->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}

	/*
	Need to find the list of points inside the selected patches
	Need to find the triangles joining the two patches
	*/

	/*
	Take the list of fringe points and delete the edges with atleast one point
	from the fringe set
	Categorize edges using depth first search
	Take user input as the initial source
	*/
	state.WASDenabled = true;
	state.patchCloud = upper_patch;
	state.patchLabel = upper_label;
	state.color = std::make_tuple(255,0,0);

	while(!v->wasStopped()){
		v->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}

	return 1;
}