#include "offline_preparation.h"

void computModelFeatureCloud(const PointCloud<PointNormal>::Ptr &modelNormalCloud,
		PointCloud<PPFSignature>::Ptr  &modelFeatureCloud,OfflineParamOut &pOut)
{
	pOut.transform_mgs.resize(modelNormalCloud->points.size());
    //改变一下模型特征点云中特征的数量,数量为|M|的给定点云，对应的modelFeatureCloud中的有效特征数量是|M|*|M|-M
    //那么为什么resize为|M|*|M| ？见后文，主要是为了方便以后的计算
    modelFeatureCloud->points.resize(modelNormalCloud->points.size()*modelNormalCloud->points.size());
    modelFeatureCloud->height=1;
    modelFeatureCloud->width=static_cast<uint32_t>(modelFeatureCloud->points.size());
    modelFeatureCloud->is_dense=true;
    PCL_INFO("本模型特征点云的有效元素数量为|M|*|M|-M =  %d \n", 
			modelFeatureCloud->points.size()-modelNormalCloud->points.size());

    //计算模型第一点的alpha_m以及  其与任意第二点之间的连续ppf描述子向量
	for (size_t i = 0; i < modelNormalCloud->points.size(); ++i)
	{
		//===计算模型第一点旋转至与world重合，法向量也重合时候的变换矩阵
	    Eigen::Affine3f transform_mg=pointpairToWorld(modelNormalCloud->points[i].getVector3fMap(), 
				modelNormalCloud->points[i].getNormalVector3fMap());
		//将当前模型参考点的变换矩阵存起来
		pOut.transform_mgs[i]=transform_mg;

		for (size_t j = 0; j < modelNormalCloud->points.size(); ++j)
		{
			PPFSignature p;
			if (i != j) //第i个点不和自身构成点对
			{
				if ( computeContinuousPPF(
					modelNormalCloud->points[i].getVector3fMap(),
					modelNormalCloud->points[i].getNormalVector3fMap(),
					modelNormalCloud->points[j].getVector3fMap(),
					modelNormalCloud->points[j].getNormalVector3fMap(),
					p.f1, p.f2, p.f3, p.f4))
				{
					//把第二点根据上面的旋转矩阵旋转到合适位置，并计算alpha_m之后将它存起来					
					p.alpha_m = computeAlpha_M_S(modelNormalCloud->points[j].getVector3fMap(),transform_mg);
				}
				else
				{
					PCL_ERROR("Computing pair feature vector between points %u and %u went wrong.\n", i, j);
					p.f1 = p.f2 = p.f3 = p.f4 = p.alpha_m = std::numeric_limits<float>::quiet_NaN();
				}
			}
			//不计算点对(i,i)
			else  
			{
				p.f1 = p.f2 = p.f3 = p.f4 = p.alpha_m = std::numeric_limits<float>::quiet_NaN();
				modelFeatureCloud->is_dense=false;
			}
			/**
			* 注意这里的做法，因为output是点云，索引只有一个值，如何将点对(i,j)记录到一个索引中？
			* 它将i，j进行了编码 i*|M|+j   但是怎么反向解算出来呢？解答见wgk_00_ppf_registration.cpp
			*/
			modelFeatureCloud->points[i*modelNormalCloud->points.size() + j] = p;
		}
	}
}


void buildModelHashtable(PointCloud<PPFSignature>::ConstPtr modelFeatureCloud, DiscretParamIn &pIn, OfflineParamOut &pOut)
{
    pOut.feature_hash_map_=boost::make_shared<FeatureHashMapType>(); //构建哈希表
    pOut.feature_hash_map_->clear();//先清空哈希表
    pOut.MaxPointpairDist=-1;//
	pOut.model_size=modelFeatureCloud->points.size();
	pOut.alpha_m.resize(pOut.model_size);



    unsigned int n = static_cast<unsigned int> (std::sqrt(static_cast<float> (modelFeatureCloud->points.size())));
    //存放离散ppf，以及其泛化描述子，实际上离散ppf每个维度都会大于零，这里使用int，只是为了去除小于0的异常值
    int d1[3]={0},
           d2[3]={0},
           d3[3]={0},
           d4[3]={0};
    uint8_t a,b,c,d;

	for (uint16_t i = 0; i < n; ++i)
	{
        //初始化一个具有n元素的容器alpha_m_row
		for (uint16_t j = 0; j < n; ++j)
		{
			if (i != j)
			{
                //解算并得到离散ppf描述子
				d1[1] = static_cast<int> (std::ceil(modelFeatureCloud->points[i*n + j].f1 / pIn.hashAngle_step));
				d2[1] = static_cast<int> (std::ceil(modelFeatureCloud->points[i*n + j].f2 / pIn.hashAngle_step));
				d3[1] = static_cast<int> (std::ceil(modelFeatureCloud->points[i*n + j].f3 /  pIn.hashAngle_step));
				d4[1] = static_cast<int> (std::ceil(modelFeatureCloud->points[i*n + j].f4 /  pIn.hashDist_step));
				// 在这里对点对(i,j)的ppf进行泛化
				d1[0] = d1[1] - 1;
				d1[2] = d1[1] + 1;
				d2[0] = d2[1] - 1;
				d2[2] = d2[1] + 1;
				d3[0] = d3[1] - 1;
				d3[2] = d3[1] + 1;
				d4[0] = d4[1] - 1;
				d4[2] = d4[1] + 1;
				// 把{(d1[*],d2[*],d3[*],d4[*]),(i,j)}作为数据对存放在哈希表中，其中(d1[*],d2[*],d3[*],d4[*])作为key，(i,j)作为值
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
                                //以离散ppf为key，向哈希表中存点对
                                pOut.feature_hash_map_->insert(std::pair<HashKeyStruct, std::pair<uint16_t, uint16_t> >(HashKeyStruct(d1[a], d2[b], d3[c], d4[d]), std::pair<uint16_t, uint16_t>(i, j)));
							}

				// 点对是mi_mj，其中mi是模型参考点，对应的alpha_m输入对应的容器中
				pOut.alpha_m[i*n + j] = modelFeatureCloud->points[i*n + j].alpha_m;
				// 在这里实现了检测给定模型的最远距离
				if (pOut.MaxPointpairDist < modelFeatureCloud->points[i*n + j].f4)
					pOut.MaxPointpairDist = modelFeatureCloud->points[i*n + j].f4;
			}
		}
	}
}


