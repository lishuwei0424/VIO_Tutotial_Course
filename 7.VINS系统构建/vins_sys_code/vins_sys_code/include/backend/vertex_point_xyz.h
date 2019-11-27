#ifndef MYSLAM_BACKEND_POINTVERTEX_H
#define MYSLAM_BACKEND_POINTVERTEX_H

#include "vertex.h"

namespace myslam
{
namespace backend
{

/*********************************************************************************************************************
 * @brief 以xyz形式参数化的顶点
 ********************************************************************************************************************/
class VertexPointXYZ : public Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPointXYZ() : Vertex(3) {}

    /*****************************************************************************************************************
    1.const c++使用的大全
    https://www.cnblogs.com/lanjianhappy/p/7298427.html

    2.C++中的常量函数
    格式：<类型说明符><函数名>(<参数表>)const;
    要注意的是，修饰符const要加在函数说明的尾部（若放在首部的话，则是对函数返回值的修饰），它是函数类型的一部分。
    *****************************************************************************************************************/
    std::string TypeInfo() const { return "VertexPointXYZ"; }
};

} // namespace backend
} // namespace myslam

#endif
