/**
 * \brief : 主要是离线阶段与在线阶段中使用到的各项公用参数以及数据类型
 */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/boost.h>

using namespace std;
using namespace pcl;


//===============================ppf 离线和在线参数=====================================
typedef boost::unordered_multimap<HashKeyStruct, std::pair<uint16_t, uint16_t> > FeatureHashMapType;//离线模型全局特征哈希表
typedef boost::shared_ptr<FeatureHashMapType> FeatureHashMapTypePtr;//离线模型全局特征哈希表的智能指针


typedef boost::unordered_map<HashKeyStruct, uint32_t> Redundancy_check_HashMapType;//hs_ppf冗余检查表
typedef boost::shared_ptr<Redundancy_check_HashMapType> Redundancy_check_HashMapTypePtr;//hs_ppf冗余检查表智能指针


//哈希表的key结构(离散的四位PPF描述子)使用两个std::pair嵌套结构实现
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

//离散化参数
struct DiscretParamIn
{
    float hashAngle_step;//哈希表的角度离散step（同时也是ppf描述子角度的离散step）
    float hashDist_step;//哈希表的距离离散step（同时也是ppf描述子距离的离散step）
    float alphaAngle_step;//投票累加器alpha角度的专用离散step
};
//对单个模型离线建模之后得到的结果参数
struct OfflineParamOut
{
    FeatureHashMapTypePtr feature_hash_map_ ;//构建的哈希表的指针
    float MaxPointpairDist;//模型点云中最远的两个点的距离
    vector<float> alpha_m;//存放点云alpha_m的容器
    uint16_t model_size;//当前model点云的点数多少
    //存放模型点云每个模型参考点旋转到world坐标系的变换关系，
    //因为在线识别反解时，还是需要这个的    
    vector<Eigen::Affine3f> transform_mgs;
    //把当前模型的点法云指针也保存下来，到在线阶段计算还要用
    //PointCloud<PointNormal>::Ptr  modelFeatureCloud;
};