/**
 * \brief 完成在线查表与投票的函数部分实现
*/

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/registration/boost.h>
#include <pthread.h>

#include"compute2points_ppf.h"
#include"pointpairToWorld.h"
#include"offline_preparation.h"
#include"param.h"

using namespace pcl;
using namespace std;

class online_matching
{

typedef boost::unordered_map<HashKeyStruct, uint32_t> Redundancy_check_HashMapType;
typedef boost::shared_ptr<Redundancy_check_HashMapType> Redundancy_check_HashMapTypePtr;

//局部坐标与对应票数结构体
struct  LocalCoodsVotes: public std::pair <std::pair <uint16_t, uint16_t>,uint16_t  > 
{
    LocalCoodsVotes(uint16_t &modelReferencePointIndex, uint16_t &discreteAlpha, uint16_t &votes)
    {
        this->first.first=modelReferencePointIndex;
        this->first.second=discreteAlpha;
        this->second=votes;
    }
};


PointCloud<PointNormal>::Ptr scene_input;
FeatureHashMapTypePtr feature_hash_map;
DiscretParamIn dp;
OfflineParamOut op;


online_matching(const PointCloud<PointNormal>::Ptr &scene_input_,const DiscretParamIn &dp_,const OfflineParamOut &op_)
{
    scene_input=scene_input_;
    //feature_hash_map = feature_hash_map_;
    dp=dp_;
    op=op_;

}


/**
 * \brief 针对某个场景参考点进行查表和投票的过程，输出一个二维累加器
 * \param [in] scene_reference_index 参考点的索引
 * \param [in] secondPoints_indices第二点的索引集合
 * \param [in_class_member] scene_input 输入类的场景点法云
*/
void  oneSceneReferance(int &scene_reference_index,vector<int> &secondPoints_indices)
{

    //构建冗余检查表
    Redundancy_check_HashMapTypePtr Redundancy_check_(new Redundancy_check_HashMapType);
    //建立存放搜索结果的容器
    vector<pair<uint16_t,uint16_t>> nearest_indices;
    //建立二维累加器
    vector <vector <uint16_t> > accumulator_array; 
    //调整累加器尺寸大小
	accumulator_array.resize(op.model_size);
    uint8_t aux_size = static_cast<uint8_t> (floor(2 * M_PI / dp.alphaAngle_step)) + 1;
    for (uint16_t i = 0; i < op.model_size; ++i)
	{
		//创建内层空间的大小与原始值
		vector<uint16_t> aux(aux_size,0);
		accumulator_array[i] = aux;
	}

    //计算s_r变换到世界坐标系时的变换矩阵
    Eigen::Affine3f transform_sg=pointpairToWorld(scene_input->points[scene_reference_index].getVector3fMap(),scene_input->points[scene_reference_index].getNormalVector3fMap());
    //处理场景点对，查表投票，获得最后的二维累加器
    for(int i=0; i<secondPoints_indices.size();++i)
    {
        int second_point_index=secondPoints_indices[i];
        float f1,f2,f3,f4;
        //计算场景ppf连续描述子
        if(computeContinuousPPF(scene_input->points[scene_reference_index].getVector3fMap(),
			scene_input->points[scene_reference_index].getNormalVector3fMap(),
			scene_input->points[second_point_index].getVector3fMap(),
			scene_input->points[second_point_index].getNormalVector3fMap(),
			f1, f2, f3, f4))
        {
            //计算alpha_s
            float alpha_s = computeAlpha_M_S(scene_input->points[second_point_index].getVector3fMap(),transform_sg);
            //查表  （并查看是否alpha_s是否重复）
            nearestNeighborSearch(f1, f2, f3, f4, alpha_s, nearest_indices, Redundancy_check_);
            //投票  （对当前查出的点对投票）
            for (std::vector<std::pair<uint16_t, uint16_t> >::iterator v_it = nearest_indices.begin(); v_it != nearest_indices.end(); ++v_it)
			{
                /**
                 * \brief alpha_m是离线阶段提前计算好的，存储在以模型点对第一点和第二点为索引，构建的容器alpha_m_中
                */
                uint16_t model_reference_index = v_it->first,  model_point_index = v_it->second;
                // 计算 alpha = alpha_m - alpha_s,  因为alpha_m和alpha_s不知道谁大谁小，但都是属于0～2pi范围内的值(相对于Y轴的正向)
                float alpha = op.alpha_m[model_reference_index*op.model_size+model_point_index] - alpha_s;
                //使得以x轴正向运用右手定则，确定正方向，然后以alpha_m为起始向量转到alpha_s, 该值范围为0~2pi (2020.08.27再次检查没错，不理解的话，简易画图看一下)
                if(alpha < 0)
                    alpha = -alpha;
                else if(alpha > 0)
                    alpha = 2*M_PI -alpha;
                //计算离散alpha角         
                uint16_t alpha_discretized = static_cast<uint16_t>(std::ceil(alpha / dp.alphaAngle_step));
                //投票
                accumulator_array[model_reference_index][alpha_discretized] ++;
			}
        }
    }





}

/**
 * \brief 根据某个ppf离散描述子，查找并返回哈希表中存放的点对索引；附带冗余检查策略
 * \param [in] f1,f2,f3,f4 输入的ppf描述子
 * \param [in] alpha_s 当前ppf对应的点对的alpha_s角度
 * \param [in] Redundancy_check 角度
 * \param [in_class_member] feature_hash_map 模型特征哈希表
 * \param [in_class_member] dp 离散参数表
*/
void nearestNeighborSearch(float &f1,float &f2,float &f3,float &f4,float &alpha_s, std::vector<std::pair<uint16_t, uint16_t>> &indices,Redundancy_check_HashMapTypePtr & Redundancy_check)
{
    int d1[3]={0},
           d2[3]={0},
           d3[3]={0},
           d4[3]={0};
    uint8_t a,b,c,d;
    uint32_t check_num=0,temp_num =1;
    //计算alpha_s偏移量
    uint8_t alpha_s_discretized = static_cast<uint8_t>(ceil((alpha_s + M_PI) / dp.alphaAngle_step));

    //解算并得到离散ppf描述子
	d1[1] = static_cast<int> (std::ceil(f1 /dp.hashAngle_step));
	d2[1] = static_cast<int> (std::ceil(f2 / dp.hashAngle_step));
	d3[1] = static_cast<int> (std::ceil(f3 /  dp.hashAngle_step));
	d4[1] = static_cast<int> (std::ceil(f4 /  dp.hashDist_step));
    // 在这里对点对(i,j)的ppf进行泛化
	d1[0] = d1[1] - 1;
	d1[2] = d1[1] + 1;
	d2[0] = d2[1] - 1;
	d2[2] = d2[1] + 1;
	d3[0] = d3[1] - 1;
	d3[2] = d3[1] + 1;
	d4[0] = d4[1] - 1;
	d4[2] = d4[1] + 1;

	for (a = 0; a<3; ++a)
		for (b = 0; b<3; ++b)
			for (c = 0; c<3; ++c)
				for (d = 0; d < 3; ++d)
				{
					//当d*[1]到达最大值，或溢出的时候，就会导致d*[2]为负值，跳过
					if ((d1[2] <0) || (d2[2] <0) || (d3[2] <0) || (d4[2] <0))
						continue;
                    else if((d1[0] <0) || (d2[0] <0) || (d3[0] <0) || (d4[0] <0)) 
                        continue;
					// 设置key
					HashKeyStruct key = HashKeyStruct(d1[a], d2[b], d3[c], d4[d]);
					// 查找重复检测哈希表中的check数是否存在，如果存在就读取出来，如果不存在就创建一个为0
					check_num = Redundancy_check->operator[](key);
                    if (((temp_num << alpha_s_discretized)&check_num) == 0)
					{
						// 得到与d1[a], d2[b], d3[c], d4[d]对应的模型点对集合容器map_iterator_pair
						auto map_iterator_pair = op.feature_hash_map_->equal_range(key);
						// 将map_iterator_pair中的模型点对 全部转移到indices中
						for (; map_iterator_pair.first != map_iterator_pair.second; ++map_iterator_pair.first)
							indices.emplace_back(map_iterator_pair.first->second.first,
								map_iterator_pair.first->second.second);

						// 将旧check_num对应的重复检查标志位置 1
						check_num |= temp_num;
						// 将该ppf处的检查数替换为新的
						Redundancy_check->operator[](key) = check_num;
					}


	    		}

}


/**
 * \brief 根据二维累加器的票数，抽取假设姿
 * \param [in]accumulator_array 二维累加器
 * \param [in]transform_sg 当前参考点旋转到原点的变换矩阵
 * \param [in_class member] op 
 */
void extractHypoPoses(vector <vector <uint16_t> > &accumulator_array, Eigen::Affine3f &transform_sg)
{
    uint16_t max_votes_=0;
    //抽取阈值
    uint8_t threshold_count = 7;
    float threshold_percent =0.8;
    //保留票数排名前threshold_count个的位姿
    list<LocalCoodsVotes> local_coods_votes_list(threshold_count);

    //票数高于阈值的局部坐标
	vector<LocalCoodsVotes> local_coods_votes;

	//先检测投票峰值的局部坐标和票数，并存起来票数排名前threshold_count个的局部坐标
    for (uint16_t i = 0; i < accumulator_array.size(); ++i)
		for (uint16_t j = 0; j < accumulator_array.back().size(); ++j)
		{
			if (accumulator_array[i][j] > max_votes_)
			{
				max_votes_ = accumulator_array[i][j];

                local_coods_votes_list.push_back(LocalCoodsVotes(i,j,max_votes_));
                local_coods_votes_list.pop_front();
			}
		}
    //判断排名靠前的票数有没有达到最高票数的0.8倍
    Eigen::Affine3f transform_mg;
    float alpha;
    for(auto it=local_coods_votes_list.begin();it!=local_coods_votes_list.end();++it)
    {
        if(it->second>threshold_percent*max_votes_)
        {
            //local_coods_votes.push_back(*it);

            //transform_mg=pointpairToWorld()
            alpha = 





        }
    }
    //






}




};