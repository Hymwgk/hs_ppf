/**
 * \brief 主要定义了几个离线建模阶段的预备函数，实现了离线建模过程的如下几个关键步骤：
 * 1. computModelFeatureCloud函数根据给定模型点法云，计算其中任意两点的连续ppf描述子，并计算出该点对对应的角度alpha_m
 * 2. 根据上述计算出的ppf描述子以及对应的角度alpha_m，来构建一个4D哈希表，作为对应模型点云的离线描述
 */
#ifndef  OFFLINE_PREPARATION_
#define OFFLINE_PREPARATION_

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/registration/boost.h>


#include "compute2points_ppf.h"
#include"pointpairToWorld.h"

#include"param.h"

using namespace pcl;
using namespace std;



/**
 * \brief 该函数根据输入模型的点法云，计算了任意两点之间的ppf描述子以及该点对的alpha_m值
 * \param modelNormalCloud [in]某模型的点法云指针
 * \param  OfflineParamOut.transform_mgs [out] 存放模型点云每个模型参考点旋转到world坐标系的变换关系的矩阵                                          
 * \param modelFeatureCloud [out]由任意点对ppf和alpha_m构成的特征点云
*/
void computModelFeatureCloud(const PointCloud<PointNormal>::Ptr &modelNormalCloud,PointCloud<PPFSignature>::Ptr  &modelFeatureCloud,OfflineParamOut &pOut);

//===============================================构建哈希表====================================================



/**
 * \brief 根据给定参数，以及特征点云，构建离线哈希表并且返回其指针，注意哈希表形式是unorder map形式
 * \param [in] modelFeatureCloud 模型的特征点云
 * \param [in] pIn 离线建模需要的参数，包括角度和距离的离散间隔
 * \param [out] out 离线建模输出的参数，包括所构建的哈希表指针、模型最大两点间距、存放alpha_m的容器
 * 
*/
void buildModelHashtable(PointCloud<PPFSignature>::ConstPtr modelFeatureCloud, DiscretParamIn &pIn, OfflineParamOut &pOut);

#endif