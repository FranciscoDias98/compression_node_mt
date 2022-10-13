#include <pcl_conversions/pcl_conversions.h>
#include <vector>

enum node_type_t { BRANCH_NODE, LEAF_NODE };


struct BoundingBox
{
    double max_x;
    double max_y;
    double max_z;
    double min_x;
    double min_y;
    double min_z;
};





enum class Octant : unsigned char {
    O1 = 0x01,	// = 0b00000001
    O2 = 0x02,	// = 0b00000010
    O3 = 0x04,	// = 0b00000100
    O4 = 0x08,	// = 0b00001000
    O5 = 0x10,	// = 0b00010000
    O6 = 0x20,	// = 0b00100000
    O7 = 0x40,	// = 0b01000000
    O8 = 0x80	// = 0b10000000
};

class Octree
{
public:
    struct OctreeNode
    {
        OctreeNode * children[8];
        OctreeNode * parent; // optional
        bool is_leaf;
        BoundingBox bound_box;
        unsigned int leafs; // <---- ?????
        std::vector<pcl::PointXYZ> points_list;
        void *usr_val;
        //Object *     Objects;
        //Vector3      Center;
        //Vector3      HalfSize;
        //uint8_t occupacy_code;
        //pcl::PointXYZ point;

    };

    Octree() {

    }
    ~Octree();

    OctreeNode *root;

    void get_AABB();
protected:
    double resolution; // NxNxN
    unsigned int depth;
    unsigned int point_count;
    unsigned int leaf_count;
    unsigned int branch_count;
    std::vector<pcl::PointXYZ> points_vector;
    std::vector<char> bit_patern;
    BoundingBox bounding_box_region;

};


