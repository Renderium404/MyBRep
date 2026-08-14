#ifndef MYBREP_GEOMETRY_CONSTRUCTION_GEOMETRY_EXTRUDED_H
#define MYBREP_GEOMETRY_CONSTRUCTION_GEOMETRY_EXTRUDED_H

#include <cstddef>
#include <vector>

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Shape/Geometry_Shape.h"

namespace MyBRep
{

// 表示由局部XY平面闭合母线区域沿局部Z轴对称拉伸形成的有限封闭连续几何体。
//
// 母线由完整Geometry_Curve及其有限参数区间共同描述，当前精确空间查询支持Line和Circle区间。
// 实体沿局部Z轴从-height/2延伸到+height/2，不依赖Topology_Edge或Topology_Wire。
class Geometry_Extruded : public Geometry_Shape
{
public:
    // 描述完整参数曲线上的一个有限有向母线段。
    struct ProfileSegment
    {
        ProfileSegment(const Foundation::RefPtr<const Geometry_Curve>& sourceCurve,
                       double sourceFirstParameter,
                       double sourceLastParameter)
            : curve(sourceCurve)
            , firstParameter(sourceFirstParameter)
            , lastParameter(sourceLastParameter)
        {
        }

        Foundation::RefPtr<const Geometry_Curve> curve; // 当前母线段引用的不可变完整参数曲线。
        double firstParameter; // 当前母线段起点参数。
        double lastParameter; // 当前母线段终点参数，允许小于firstParameter表示反向使用。
    };

public:
    // 使用有序闭合母线段、完整拉伸高度和几何容差创建标准拉伸体。
    Geometry_Extruded(const std::vector<ProfileSegment>& profileSegments,
                      double height,
                      double profileTolerance);

    /// 母线几何数据

    // 返回闭合母线包含的有限曲线段数量。
    std::size_t profileSegmentCount() const;
    // 返回指定编号的母线段描述。
    const ProfileSegment& profileSegment(std::size_t index) const;
    // 返回完整有序闭合母线段序列。
    const std::vector<ProfileSegment>& profileSegments() const;
    // 返回母线连接、平面和边界判断使用的几何容差。
    double profileTolerance() const;
    // 返回母线区域在局部XY平面中的轴对齐包围盒。
    const Bounds3& profileBounds() const;
    // 返回闭合母线区域的有符号面积，逆时针为正，顺时针为负。
    double profileSignedArea() const;

    /// 拉伸参数

    // 返回拉伸体完整高度。
    double height() const;

    /// 几何属性

    // 返回拉伸连续几何体类型。
    ShapeKind kind() const override;

    /// 标准空间查询

    // 判断指定局部三维点是否位于拉伸体内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point) const override;
    // 当前拉伸体在受支持母线类型下提供精确局部有符号距离查询。
    bool supportsSignedDistance() const override;
    // 返回指定局部点到有限封闭拉伸体边界的精确有符号距离。
    double signedDistanceLocalPoint(const MyMath::Vector3& point) const override;
    // 返回指定局部轴对齐包围盒与拉伸体之间的保守空间关系。
    ShapeRelation classifyLocalBounds(const Bounds3& bounds) const override;

protected:
    // 通过侵入式引用计数管理拉伸连续几何体生命周期。
    ~Geometry_Extruded() override = default;

private:
    // 校验全部母线段的数据、平面约束和闭合连接关系。
    void validateProfile() const;
    // 建立母线包围盒、有符号面积和实体局部包围盒。
    void rebuildCaches();
    // 返回指定母线段起点。
    MyMath::Vector3 segmentStartPoint(const ProfileSegment& segment) const;
    // 返回指定母线段终点。
    MyMath::Vector3 segmentEndPoint(const ProfileSegment& segment) const;
    // 判断指定局部XY平面点是否位于母线区域内部或边界上。
    bool containsProfilePoint(const MyMath::Vector3& point, double tolerance) const;
    // 返回指定局部XY平面点到母线边界的精确无符号距离。
    double profileBoundaryDistance(const MyMath::Vector3& point) const;
    // 返回指定局部XY平面矩形与母线区域之间的保守空间关系。
    ShapeRelation classifyProfileBounds(const Bounds3& bounds) const;

private:
    std::vector<ProfileSegment> m_profileSegments; // 按轮廓方向排列的有限母线段。
    double m_height; // 拉伸体完整高度。
    double m_halfHeight; // 拉伸体半高度。
    double m_profileTolerance; // 母线连接、平面和边界判断使用的几何容差。
    Bounds3 m_profileBounds; // 母线区域在局部XY平面中的轴对齐包围盒。
    double m_profileSignedArea; // 闭合母线区域有符号面积。
};

}

#endif // MYBREP_GEOMETRY_CONSTRUCTION_GEOMETRY_EXTRUDED_H