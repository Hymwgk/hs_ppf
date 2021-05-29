/**
 * \brief: 写了一个ppf描述子的计算模块，主要可完成的功能有：
 * 1.计算原始的连续ppf描述子
 * 2.根据连续的ppf描述子，计算出离散的ppf描述子（没写，感觉不是很有必要）
 * 
 */
#ifndef COMPUTE2POINTS_PPF_
#define COMPUTE2POINTS_PPF_

#include <Eigen/Core>
#include <Eigen/Eigen>

/**
* \brief：输入两个三维向量，返回两者之间的角度
* \param [in] a 某指定的三维向量
* \param [in] b 某指定的三维向量
* \return  a b 之间的夹角，范围是(0 , pi ]
*/
inline float angleBetween(const Eigen::Vector3f& a, const Eigen::Vector3f& b);


/**
* \brief   函数computeContinuousPPF
*				根据两点以及其表面法向量来计算连续该点对的连续PPF描述子向量
* \param [in] p1 点对第一点的三维坐标向量
* \param [in] n1 点对第一点对应的法向量坐标
* \param [in] p2 点对第二点的三维坐标
* \param [in] n2 点对第二点对应的法向量坐标
* \param [out] f1   n1与两点连线的夹角
* \param [out] f2   n2与两点连线的夹角
* \param [out] f3   n1与n2之间的夹角
* \param [out] f4    p1 , p2之间的距离
* \return 是否计算成功的标志位，bool类型
*/
bool computeContinuousPPF(const Eigen::Vector3f &p1, const Eigen::Vector3f &n1,const Eigen::Vector3f &p2, const Eigen::Vector3f &n2,
			float &f1, float &f2, float &f3, float &f4);






#endif
