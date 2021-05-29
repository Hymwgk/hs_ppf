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

//结构体：记录下某个局部坐标(i,j)以及其对应的票数
struct  LocalCoodsVotes: public std::pair <std::pair <uint16_t, uint16_t>,uint16_t> 
{
    
    LocalCoodsVotes(uint16_t &modelReferencePointIndex, 
            uint16_t &discreteAlpha, uint16_t &votes)
    {
        this->first.first=modelReferencePointIndex; //
        this->first.second=discreteAlpha;
        this->second=votes;
    }
};


PointCloud<PointNormal>::Ptr scene_input;//场景点云
FeatureHashMapTypePtr feature_hash_map;
DiscretParamIn dp;
OfflineParamOut op;//单个模型离线建模输出的参数，但是我感觉，应该输入一个容器，一下子装所有的模型的参数
                                            //后来再改


online_matching(const PointCloud<PointNormal>::Ptr &scene_input_,const DiscretParamIn &dp_,const OfflineParamOut &op_)
{
    scene_input=scene_input_;
    //feature_hash_map = feature_hash_map_;
    dp=dp_;
    printf("此处需要改动! \n");
    op=op_;

}


/**
 * \brief 针对某单个场景参考点进行查表和投票的过程，输出一个二维累加器
 * \param [in] scene_reference_index 参考点的索引
 * \param [in] secondPoints_indices 第二点的索引集合
 * \param [in] scene_input 输入类的场景点法云
*/
void  SceneReferanceVote(int &scene_reference_index,vector<int> &secondPoints_indices)
{

    //构建冗余检查表，每个参考点的冗余检查表都是独立的
    Redundancy_check_HashMapTypePtr Redundancy_check_(new Redundancy_check_HashMapType);
    //建立存放搜索结果的容器
    vector<pair<uint16_t,uint16_t>> nearest_indices;
    //建立二维投票累加器
    vector <vector <uint16_t> > accumulator_array; 
    //调整累加器尺寸大小，累加器外层(行数)大小为模型点数
	accumulator_array.resize(op.model_size);
    //设置累加器内层(列数)大小 = 2pi/step+1  向下取整
    uint8_t aux_size = static_cast<uint8_t> (floor(2 * M_PI / dp.alphaAngle_step)) + 1;
    //为累加器内部值进行初始化(清0)
    for (uint16_t i = 0; i < op.model_size; ++i)
	{
		//创建内层空间的大小与原始值
		vector<uint16_t> aux(aux_size,0);
		accumulator_array[i] = aux;
	}

    //计算s_r变换到世界坐标系时的变换矩阵
    Eigen::Affine3f transform_sg=pointpairToWorld(scene_input->points[scene_reference_index].getVector3fMap(),scene_input->points[scene_reference_index].getNormalVector3fMap());
    //处理场景点对，查表投票，获得最后的二维累加器
    for(uint32_t i=0; i<secondPoints_indices.size();++i)
    {
        //获取场景第二点索引
        int second_point_index=secondPoints_indices[i];
        //连续ppf描述子声明
        float ppf[4];
        //计算场景ppf连续描述子
        if(computeContinuousPPF(scene_input->points[scene_reference_index].getVector3fMap(),
			scene_input->points[scene_reference_index].getNormalVector3fMap(),
			scene_input->points[second_point_index].getVector3fMap(),
			scene_input->points[second_point_index].getNormalVector3fMap(),
			ppf[0], ppf[1],ppf[2], ppf[3]))
        {
            //计算alpha_s
            float alpha_s = computeAlpha_M_S(scene_input->points[second_point_index].getVector3fMap(),transform_sg);
            //查表  （包括冗余检查）
            nearestNeighborSearch(ppf, dp.hashAngle_step, dp.hashDist_step, alpha_s, nearest_indices, Redundancy_check_);
            //投票  （对当前查出的点对投票）
            for (std::vector<std::pair<uint16_t, uint16_t> >::iterator v_it = nearest_indices.begin(); v_it != nearest_indices.end(); ++v_it)
			{
                /**
                 * \brief alpha_m是离线阶段提前计算好的，存储在以模型点对第一点和第二点为索引，构建的容器alpha_m_中
                */
                uint16_t model_reference_index = v_it->first,  model_point_index = v_it->second;
                // 计算 alpha = alpha_m - alpha_s,  因为alpha_m和alpha_s不知道谁大谁小，但都是属于0~2pi范围内的值(相对于Y轴的正向)
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
        }//此处完成了一个点对的投票过程(还没有聚类等)
        else 
            //ppf描述子计算报错
            PCL_ERROR("Computing pair feature vector between points %u and %u went wrong.\n",
                     scene_reference_index, second_point_index);
    }//此处完成了对该场景参考点，与所有第二点构成的点对的投票过程(还没有聚类等)
}





/**
 * \brief 根据某个ppf离散描述子，查找并返回模型哈希表中对应存放的点对索引；附带冗余检查策略
 * \param [in] ppf[4]  输入的场景点对的连续ppf描述子
 * \param [in] angle_step 离散ppf使用的角度离散间隔
 * \param [in] dist_step  离散ppf使用的距离离散间隔
 * \param [in] alpha_s  当前ppf对应的点对的alpha_s角度
 * \param [in] Redundancy_check 当前场景参考点(第一点)对应的冗余检查表
 * \param [in] feature_hash_map   该模型特征哈希表
 * \param [out] indices 模型哈希表中与该ppf相匹配的模型点对索引集合 
*/
void nearestNeighborSearch(float ppf[4], float &angle_step, float &dist_step,
        float &alpha_s, std::vector<std::pair<uint16_t, uint16_t>> &indices,Redundancy_check_HashMapTypePtr & Redundancy_check)
{
    //
    int d1[3]={0},
           d2[3]={0},
           d3[3]={0},
           d4[3]={0};
    uint8_t a,b,c,d;
    //
    uint32_t check_num=0,off_set =1;
    //计算alpha_s偏移量
    uint8_t alpha_s_discretized = static_cast<uint8_t>(ceil((alpha_s + M_PI) / dp.alphaAngle_step));

    //解算并得到离散ppf描述子
	d1[1] = static_cast<int> (std::ceil(ppf[0] /angle_step));
	d2[1] = static_cast<int> (std::ceil(ppf[1]/angle_step));
	d3[1] = static_cast<int> (std::ceil(ppf[2]/angle_step));
	d4[1] = static_cast<int> (std::ceil(ppf[3]/dist_step));
    // 在这里对点对(i,j)的ppf进行泛化
	d1[0] = d1[1] - 1;
	d1[2] = d1[1] + 1;
	d2[0] = d2[1] - 1;
	d2[2] = d2[1] + 1;
	d3[0] = d3[1] - 1;
	d3[2] = d3[1] + 1;
	d4[0] = d4[1] - 1;
	d4[2] = d4[1] + 1;

    //开始对泛化（包括原始的）离散PPF描述子进行如下操作：
    //1.使用检查是否冗余
    //2.(非冗余时)检索模型哈希表中与场景离散PPF对应的模型点对(索引)集合
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
					// 设置key=离散PPF
					HashKeyStruct key = HashKeyStruct(d1[a], d2[b], d3[c], d4[d]);
					// 查找重复检测哈希表中的check数是否存在，如果存在就读取出来，如果不存在就创建一个为0
					check_num = Redundancy_check->operator[](key);
                    //保证off_set复位为1
                    off_set=1;
                    //如果通过了冗余检查，开始读取哈希表格中与该PPF对应的模型点对索引集合
                    if (((off_set << alpha_s_discretized)&check_num) == 0)
					{
						// 得到与d1[a], d2[b], d3[c], d4[d]对应的模型点对集合容器map_iterator_pair
						auto map_iterator_pair = op.feature_hash_map_->equal_range(key);
						// 将map_iterator_pair中的模型点对 全部转移到indices中
						for (; map_iterator_pair.first != map_iterator_pair.second; ++map_iterator_pair.first)
							indices.emplace_back(map_iterator_pair.first->second.first,
								map_iterator_pair.first->second.second);

						// 将旧check_num对应的重复检查标志位置 1
						check_num |= off_set;
						// 将该ppf处的检查数替换为新的
						Redundancy_check->operator[](key) = check_num;
					}
                    else
                    {
                        PCL_DEBUG("未通过冗余检查！%d", check_num);
                    }
	    		}
}


/**
 * \brief    根据(对某个场景参考点投票得出的)二维累加器以及票数，排序得到票数峰值，反解出假设姿态
 * \param [in] accumulator_array 二维累加器
 * \param [in] transform_sg 当前参考点旋转到原点的变换矩阵
 * \param [in_class member] 
 * \param [in_class member] op  目标模型的离线计算参数
 * \param [out] VotesTransformForOneThread  当前场景参考点所属的子线程中，存放所有参考点计算出的"旋转+投票"的容器
 */
void extractHypoPoses(vector <vector <uint16_t> > &accumulator_array, 
        Eigen::Affine3f &transform_sg,vector<std::pair<Eigen::Affine3f,uint16_t>> &TransformVotesForOneThread )
{
    uint16_t max_votes_=0;
    //抽取阈值
    uint8_t threshold_count = 7;
    float threshold_percent =0.8;
    //设置一个指定长度的list，内装数据类型为LocalCoodsVotes
    list<LocalCoodsVotes> LocalCoodsVotesListForOneSr(threshold_count);

    //票数高于阈值的局部坐标
	vector<LocalCoodsVotes> local_coods_votes;
	//先检测投票峰值的局部坐标和票数，并存起来票数排名前threshold_count个的局部坐标
    for (uint16_t i = 0; i < accumulator_array.size(); ++i)
		for (uint16_t j = 0; j < accumulator_array.back().size(); ++j)
		{
			if (accumulator_array[i][j] > max_votes_)
			{
				max_votes_ = accumulator_array[i][j];
                //不断让更高票数的(i,j,votes)从底部压入，然后把低票数的(i,j,votes)弹出，最终将会保证list中留下根据votes
                //从低到高排序的threshold_count个(i,j,votes)
                LocalCoodsVotesListForOneSr.push_back(LocalCoodsVotes(i,j,max_votes_)); //压入一个值
                LocalCoodsVotesListForOneSr.pop_front();//就弹出一个，保证list内部元素数量不变
			}
		}

    //
    Eigen::Affine3f transform_mg,transform;
    //
    float alpha;
    //遍历判断list中各个元素的票数有没有达到最高票数的0.8倍(即阈值)
    for(auto it=LocalCoodsVotesListForOneSr.begin();it!=LocalCoodsVotesListForOneSr.end();++it)
    {
        //如果超过了阈值，就进一步反解出
        if(it->second > threshold_percent*max_votes_)
        {
            //读取离线阶段计算和保存的  该模型第一点到世界坐标的旋转
            transform_mg=op.transform_mgs[it->first.first];
            //解算出alpha值
            alpha = static_cast<float>(it->first.second*dp.alphaAngle_step + 0.5*dp.alphaAngle_step);
            //解算出最终的变换矩阵
            transform = ModelToScene(transform_mg,transform_sg,alpha);
            //把变换矩阵和对应的票数存到所属线程的大容器中
            TransformVotesForOneThread.emplace_back(std::pair<Eigen::Affine3f,uint16_t>(transform,it->second));
        }
        else//没有超过阈值就舍弃           
            PCL_DEBUG("局部坐标%u,%u处的投票值低于阈值.\n",
                     it->first.first, it->first.second);
    }//结束了单个Sr的投票和姿态计算
}




};