#include <algorithm>
#include <cmath>
#include <vector>

#include <QApplication>
#include <QDebug>
#include <QTimer>
#include "MyMath/CoordinateSystem.h"
#include "MyMath/MathUtils.h"
#include "MyMath/Quaternion.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Instance/Solid.h"
#include "MyBRep/Modeling/Edge/EdgeModeling.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Shell/ShellModeling.h"
#include "MyBRep/Modeling/Solid/SolidModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Geometry/Surface/Geometry_BSplineSurface.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

class profile 
{
public:
    profile() = default;
    virtual ~profile() = default;

    // 返回弧长 l 处的二维法向量（在 r-h 平面内），已归一化
    virtual MyMath::Vector2 normalAtL(double l) const = 0;
    // 曲线总弧长
    virtual double length() const = 0;
    // 弧长 l 处的 r 坐标
    virtual double r(double l) const = 0;
    // 弧长 l 处的 h 坐标
    virtual double h(double l) const = 0;
};
class Line : public profile
{
public:
    Line(const MyMath::Vector2& start, const MyMath::Vector2& end)
        : start_(start), end_(end)
    {
        MyMath::Vector2 dir = end_ - start_;
        length_ = dir.length();
        if (length_ > 1e-12) {
            tangent_ = dir / length_;                 // 单位切向量
        } else {
            tangent_ = MyMath::Vector2(0.0, 0.0);     // 退化情况
        }
        // 对于弧长参数，切线 = (r', h')，法向量 = (h', -r')
        normal_ = MyMath::Vector2(tangent_.y(), -tangent_.x());
    }

    MyMath::Vector2 normalAtL(double l) const override
    {
        // 直线法向量处处相同
        return normal_;
    }

    double length() const override
    {
        return length_;
    }

    double r(double l) const override
    {
        return start_.x() + tangent_.x() * l;
    }

    double h(double l) const override
    {
        return start_.y() + tangent_.y() * l;
    }

private:
    MyMath::Vector2 start_;
    MyMath::Vector2 end_;
    MyMath::Vector2 tangent_;
    MyMath::Vector2 normal_;
    double length_;
};
class Arc : public profile
{
public:
    Arc(const MyMath::Vector2& center, double radius,
        double startAngle, double endAngle)
        : center_(center), radius_(radius),
          startAngle_(startAngle), endAngle_(endAngle)
    {
        // 计算弧长（假设角度为弧度，逆时针为正）
        double deltaAngle = endAngle_ - startAngle_;
        length_ = std::abs(deltaAngle) * radius_;
        // 记录角度方向
        angleDir_ = (deltaAngle >= 0) ? 1.0 : -1.0;
    }

    MyMath::Vector2 normalAtL(double l) const override
    {
        double theta = startAngle_ + angleDir_ * (l / radius_);
        // 对于逆时针弧，外法线指向远离圆心方向
        return MyMath::Vector2(std::cos(theta), std::sin(theta));
    }

    double length() const override
    {
        return length_;
    }

    double r(double l) const override
    {
        double theta = startAngle_ + angleDir_ * (l / radius_);
        return center_.x() + radius_ * std::cos(theta);
    }

    double h(double l) const override
    {
        double theta = startAngle_ + angleDir_ * (l / radius_);
        return center_.y() + radius_ * std::sin(theta);
    }

private:
    MyMath::Vector2 center_;
    double radius_;
    double startAngle_;
    double endAngle_;
    double length_;
    double angleDir_; // +1 表示逆时针，-1 表示顺时针
};





int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    MyBRep::Display::BRepViewerWidget window;
    window.resize(1000, 700);
    window.show();
    return app.exec();
}



