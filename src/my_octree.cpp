#include "my_octree.h"
#define MAX_DEPTH 16


Octree::Octree()
{
    root = new OctreeNode;
    root->leafs = 0;
    root->is_leaf = false;



    for(int i=0;i<8;i++)
        root->children[i] = 0;


    resolution = 0.03;
    depth = 0;


}

Octree::Octree(OctreeNode node, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{

    for (auto &point : *input_cloud) {
        root->points_list.push_back(point);
    }




}

Octree::~Octree(){
    delete root;
}

void Octree::build_octree()
{
    //declarations

    if(points_vector.size() <= 1)
        return;



}


