/**
 * \brief : 主要是离线阶段与在线阶段中使用到的各项公用参数
 */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/boost.h>

using namespace std;
using namespace pcl;


//===============================ppf 离线在线参数=====================================
typedef boost::unordered_multimap<HashKeyStruct, std::pair<uint16_t, uint16_t> > FeatureHashMapType;
typedef boost::shared_ptr<FeatureHashMapType> FeatureHashMapTypePtr;


//哈希表的key结构
struct HashKeyStruct : public std::pair <int, std::pair <int, std::pair <int, int> > >
{
    HashKeyStruct(int a, int b, int c, int d)
    {
        this->first = a;
        this->second.first = b;
        this->second.second.first = c;
        this->second.second.second = d;
    }
};

//离线建模需要的参数
struct DiscretParamIn
{
    float hashAngle_step;
    float hashDist_step;
    float alphaAngle_step;
};
//离线建模好输出的结果参数
struct OfflineParamOut
{
    //构建的哈希表的指针
    FeatureHashMapTypePtr feature_hash_map_ ;
    //模型点云中最远的两个点的距离
    float MaxPointpairDist;
    //存放点云alpha_m的容器
    vector<float> alpha_m;
    //当前model点云的点数多少
    uint16_t model_size;
    //存放模型点云每个模型参考点旋转到world坐标系的变换关系
    vector<Eigen::Affine3f> transform_mgs;
};