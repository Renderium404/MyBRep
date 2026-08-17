#ifndef MYBREP_GEOMETRY_CONSTRUCTION_GEOMETRY_REVOLVED_H
#define MYBREP_GEOMETRY_CONSTRUCTION_GEOMETRY_REVOLVED_H

#include <cstddef>
#include <vector>

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Shape/Geometry_Shape.h"

namespace MyBRep
{

// 表示由局部XY平面闭合母线区域映射到径向-Z截面并绕局部Z轴完整旋转形成的有限封闭连续体。
//
// 母线X表示带符号旋转半径，母线Y表示实体局部Z坐标；母线必须闭合、具有非零面积并整体位于局部Y轴同一侧。
// 母线由完整Geometry_Curve及其有限有向参数区间共同描述，当前精确空间查询支持Line和Circle区间。
class Geometry_Revolved : public Geometry_Shape
{
public:
    // 描述完整参数曲线上的一个有限有向回转母线段。
    struct ProfileSegment
    {
        ProfileSegment(const Foundation::RefPtr<const Geometry_Curve>& sourceCurve, double sourceFirstParameter, double sourceLastParameter) : curve(sourceCurve)
            , firstParameter(sourceFirstParameter) , lastParameter(sourceLastParameter)
        {
        }

        Foundation::RefPtr<const Geometry_Curve> curve; // 当前母线段引用的不可变完整参数曲线。
        double firstParameter; // 当前母线段起点参数。
        double lastParameter; // 当前母线段终点参数，允许小于firstParameter表示反向使用。
    };

public:
    // 使用有序闭合有限母线段和几何容差创建绕局部Z轴完整旋转的连续实体。
    Geometry_Revolved(const std::vector<ProfileSegment>& profileSegments, double profileTolerance);
    // 通过侵入式引用计数管理回转连续体几何生命周期。
    ~Geometry_Revolved() override = default;
    /// 母线几何数据

    // 返回闭合母线包含的有限曲线段数量。
    std::size_t profileSegmentCount() const;
    // 返回指定编号的有限母线段描述。
    const ProfileSegment& profileSegment(std::size_t index) const;
    // 返回完整有序闭合母线段序列。
    const std::vector<ProfileSegment>& profileSegments() const;
    // 返回母线连接、平面和边界判断使用的几何容差。
    double profileTolerance() const;
    // 返回母线区域在局部XY平面中的轴对齐包围盒。
    const Bounds3& profileBounds() const;
    // 返回闭合母线区域的有符号面积，逆时针为正，顺时针为负。
    double profileSignedArea() const;
    // 返回母线X映射到非负旋转半径时使用的方向符号，右侧为1，左侧为-1。
    double radialSign() const;

    /// 几何属性

    // 返回完整回转连续体类型。
    ShapeKind kind() const override;

    /// 标准空间查询

    // 判断指定局部三维点是否位于回转体内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point) const override;
    // 当前回转体在受支持母线类型下提供精确局部有符号距离查询。
    bool supportsSignedDistance() const override;
    // 返回指定局部点到回转体边界的精确有符号距离。
    double signedDistanceLocalPoint(const MyMath::Vector3& point) const override;
    // 返回指定局部轴对齐包围盒与回转体之间的保守空间关系。
    ShapeRelation classifyLocalBounds(const Bounds3& bounds) const override;

    /// 快速空间查询

    // 使用已经计算好的局部包围盒中心和半尺寸执行保守分类。
    ShapeRelation classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const override;

protected:


private:
    // 校验全部有限母线段的数据、平面约束、闭合连接关系和单侧径向约束。
    void validateProfile() const;
    // 建立母线包围盒、有符号面积、径向方向和实体局部包围盒缓存。
    void rebuildCaches();
    // 返回指定母线段起点投影到严格局部XY平面后的点。
    MyMath::Vector3 segmentStartPoint(const ProfileSegment& segment) const;
    // 返回指定母线段终点投影到严格局部XY平面后的点。
    MyMath::Vector3 segmentEndPoint(const ProfileSegment& segment) const;
    // 判断指定局部XY平面点是否位于母线区域内部或边界上。
    bool containsProfilePoint(const MyMath::Vector3& point, double tolerance) const;
    // 返回指定局部XY平面点到母线边界的精确无符号距离。
    double profileBoundaryDistance(const MyMath::Vector3& point) const;
    // 返回指定局部XY平面矩形与母线区域之间的保守空间关系。
    ShapeRelation classifyProfileBounds(const Bounds3& bounds) const;
    // 使用三维AABB范围执行回转对称降维分类。
    ShapeRelation classifyRange(const MyMath::Vector3& minimum, const MyMath::Vector3& maximum) const;

private:
    std::vector<ProfileSegment> m_profileSegments; // 按轮廓方向排列的有限母线段。
    double m_profileTolerance; // 母线连接、平面和边界判断使用的几何容差。
    Bounds3 m_profileBounds; // 母线区域在局部XY平面中的轴对齐包围盒。
    double m_profileSignedArea; // 闭合母线区域的有符号面积。
    double m_radialSign; // 母线X映射到非负旋转半径时使用的方向符号。
};

}

#endif // MYBREP_GEOMETRY_CONSTRUCTION_GEOMETRY_REVOLVED_H