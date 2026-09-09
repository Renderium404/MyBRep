#ifndef MYBREPOPENGL_VIEWER_BREPVIEWERWIDGET_H
#define MYBREPOPENGL_VIEWER_BREPVIEWERWIDGET_H

#include <cstddef>
#include <map>

#include <QString>

#include "MyMath/Matrix4.h"
#include "MyBRep/Instance/Edge.h"
#include "MyBRep/Instance/Face.h"
#include "MyBRep/Instance/Shell.h"
#include "MyBRep/Instance/Solid.h"
#include "MyBRep/Instance/Wire.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"
#include "MyBRep/Topology/Solid/Topology_Solid.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"
#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyOpenGL/Viewer/OpenGLViewerWidget.h"

class BufferGeometry;

namespace MyBRep
{
namespace Display
{

// MyBRep专用Viewer。
// 继承MyOpenGL的OpenGLViewerWidget，但不修改MyOpenGL源码；B-Rep显示对象的构建、注册和释放全部在本类内部闭环。
class BRepViewerWidget : public OpenGLViewerWidget
{
public:
    explicit BRepViewerWidget(QWidget* parent = 0);
    ~BRepViewerWidget() override;

    /// Topology Edge

    BRepDisplayId addWireframe(const Topology_Edge& edge, const QString& name = "BRepEdge", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Topology_Edge& edge, const MyMath::Matrix4& localToWorld, const QString& name = "BRepEdge",
                               const BRepDisplayStyle& style = BRepDisplayStyle());

    /// Topology Wire

    BRepDisplayId addWireframe(const Topology_Wire& wire, const QString& name = "BRepWire", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Topology_Wire& wire, const MyMath::Matrix4& localToWorld, const QString& name = "BRepWire",
                               const BRepDisplayStyle& style = BRepDisplayStyle());

    /// Topology Face

    // 只显示Face全部trimming Wire边界。
    BRepDisplayId addWireframe(const Topology_Face& face, const QString& name = "BRepFace", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Topology_Face& face, const MyMath::Matrix4& localToWorld, const QString& name = "BRepFace",
                               const BRepDisplayStyle& style = BRepDisplayStyle());

    // 显示Face三角表面和全部B-Rep边界；当前第一阶段支持PlanarFaceMesher能够处理的平面Face。
    BRepDisplayId addFace(const Topology_Face& face, const QString& name = "BRepFace", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addFace(const Topology_Face& face, const MyMath::Matrix4& localToWorld, const QString& name = "BRepFace",
                          const BRepDisplayStyle& style = BRepDisplayStyle());

    /// Topology Shell

    // 只显示Shell全部唯一Topology_Edge。
    BRepDisplayId addWireframe(const Topology_Shell& shell, const QString& name = "BRepShell", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Topology_Shell& shell, const MyMath::Matrix4& localToWorld, const QString& name = "BRepShell",
                               const BRepDisplayStyle& style = BRepDisplayStyle());

    // 显示Shell统一三角表面和去重B-Rep边界；当前要求Shell全部Face均可由BRepFaceBuilder离散。
    BRepDisplayId addShell(const Topology_Shell& shell, const QString& name = "BRepShell", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addShell(const Topology_Shell& shell, const MyMath::Matrix4& localToWorld, const QString& name = "BRepShell",
                           const BRepDisplayStyle& style = BRepDisplayStyle());

    /// Topology Solid

    // 只显示Solid全部Shell范围内的唯一Topology_Edge。
    BRepDisplayId addWireframe(const Topology_Solid& solid, const QString& name = "BRepSolid", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Topology_Solid& solid, const MyMath::Matrix4& localToWorld, const QString& name = "BRepSolid",
                               const BRepDisplayStyle& style = BRepDisplayStyle());

    // 显示Solid统一三角表面和全局去重B-Rep边界；当前要求全部Shell中的Face均可由BRepFaceBuilder离散。
    BRepDisplayId addSolid(const Topology_Solid& solid, const QString& name = "BRepSolid", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addSolid(const Topology_Solid& solid, const MyMath::Matrix4& localToWorld, const QString& name = "BRepSolid",
                           const BRepDisplayStyle& style = BRepDisplayStyle());

    /// Instance

    BRepDisplayId addWireframe(const Edge& edge, const QString& name = "BRepEdge", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Wire& wire, const QString& name = "BRepWire", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Face& face, const QString& name = "BRepFace", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Shell& shell, const QString& name = "BRepShell", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Solid& solid, const QString& name = "BRepSolid", const BRepDisplayStyle& style = BRepDisplayStyle());

    // 使用Face实例自身localToWorld显示三角表面和B-Rep边界。
    BRepDisplayId addFace(const Face& face, const QString& name = "BRepFace", const BRepDisplayStyle& style = BRepDisplayStyle());

    // 使用Shell实例自身localToWorld显示统一三角表面和去重B-Rep边界。
    BRepDisplayId addShell(const Shell& shell, const QString& name = "BRepShell", const BRepDisplayStyle& style = BRepDisplayStyle());

    // 使用Solid实例自身localToWorld显示统一三角表面和全局去重B-Rep边界。
    BRepDisplayId addSolid(const Solid& solid, const QString& name = "BRepSolid", const BRepDisplayStyle& style = BRepDisplayStyle());

    /// 显示对象管理

    bool containsDisplay(BRepDisplayId id) const;
    std::size_t displayCount() const;
    BRepDisplayObject display(BRepDisplayId id) const;
    bool removeDisplay(BRepDisplayId id);
    bool clearBRepDisplays();

private:
    BRepDisplayId attachWireframe(BufferGeometry* geometry, const QString& name, const BRepDisplayStyle& style);
    BRepDisplayId attachSurfaceDisplay(BufferGeometry* surfaceGeometry, BufferGeometry* wireframeGeometry, const QString& name,
                                       const BRepDisplayStyle& style);
    bool removeDisplayResources(BRepDisplayObject& object);
    BRepDisplayId allocateDisplayId();

private:
    typedef std::map<BRepDisplayId, BRepDisplayObject> DisplayMap;

    DisplayMap m_displays;         // 当前由BRepViewerWidget创建并管理的全部B-Rep显示对象。
    BRepDisplayId m_nextDisplayId; // 下一个可分配显示对象ID。
};

}
}

#endif // MYBREPOPENGL_VIEWER_BREPVIEWERWIDGET_H
