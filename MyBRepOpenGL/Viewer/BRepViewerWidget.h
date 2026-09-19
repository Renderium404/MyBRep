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
#include "MyBRepOpenGL/Builder/BRepSolidBuilder.h"
#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Display/BRepSolidGeometryResource.h"
#include "MyOpenGL/Viewer/OpenGLViewerWidget.h"

class AxisAlignedBoundingBox;
class BufferGeometry;
class LightManager;
class QQuaternion;
class QVector3D;

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

    BRepDisplayId addWireframe(const Topology_Face& face, const QString& name = "BRepFace", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Topology_Face& face, const MyMath::Matrix4& localToWorld, const QString& name = "BRepFace",
                               const BRepDisplayStyle& style = BRepDisplayStyle());

    BRepDisplayId addFace(const Topology_Face& face, const QString& name = "BRepFace", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addFace(const Topology_Face& face, const MyMath::Matrix4& localToWorld, const QString& name = "BRepFace",
                          const BRepDisplayStyle& style = BRepDisplayStyle());

    /// Topology Shell

    BRepDisplayId addWireframe(const Topology_Shell& shell, const QString& name = "BRepShell", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Topology_Shell& shell, const MyMath::Matrix4& localToWorld, const QString& name = "BRepShell",
                               const BRepDisplayStyle& style = BRepDisplayStyle());

    BRepDisplayId addShell(const Topology_Shell& shell, const QString& name = "BRepShell", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addShell(const Topology_Shell& shell, const MyMath::Matrix4& localToWorld, const QString& name = "BRepShell",
                           const BRepDisplayStyle& style = BRepDisplayStyle());

    /// Topology Solid

    BRepDisplayId addWireframe(const Topology_Solid& solid, const QString& name = "BRepSolid", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Topology_Solid& solid, const MyMath::Matrix4& localToWorld, const QString& name = "BRepSolid",
                               const BRepDisplayStyle& style = BRepDisplayStyle());

    // 对可由MyOpenGL RenderItem TRS表达的放置，复用Topology_Solid对应的共享局部Geometry资源并创建独立RenderItem。
    // 含Shear等当前RenderItem无法表达的一般仿射放置继续回退到旧的Geometry烘焙路径。
    BRepDisplayId addSolid(const Topology_Solid& solid, const QString& name = "BRepSolid", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addSolid(const Topology_Solid& solid, const MyMath::Matrix4& localToWorld, const QString& name = "BRepSolid",
                           const BRepDisplayStyle& style = BRepDisplayStyle());

    /// Instance

    BRepDisplayId addWireframe(const Edge& edge, const QString& name = "BRepEdge", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Wire& wire, const QString& name = "BRepWire", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Face& face, const QString& name = "BRepFace", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Shell& shell, const QString& name = "BRepShell", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Solid& solid, const QString& name = "BRepSolid", const BRepDisplayStyle& style = BRepDisplayStyle());

    BRepDisplayId addFace(const Face& face, const QString& name = "BRepFace", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addShell(const Shell& shell, const QString& name = "BRepShell", const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addSolid(const Solid& solid, const QString& name = "BRepSolid", const BRepDisplayStyle& style = BRepDisplayStyle());

    /// 全局离散参数

    // 返回当前Viewer统一使用的B-Rep离散构建参数。
    const BRepSolidBuildOptions& buildOptions() const;

    // 设置当前Viewer统一使用的B-Rep离散构建参数；存在显示对象时拒绝修改。
    bool setBuildOptions(const BRepSolidBuildOptions& options);

    /// 显示对象管理

    bool containsDisplay(BRepDisplayId id) const;
    std::size_t displayCount() const;
    BRepDisplayObject display(BRepDisplayId id) const;
    bool removeDisplay(BRepDisplayId id);
    bool clearBRepDisplays();

private:
    // 返回BufferGeometry中Position属性的值偏移，不存在时返回-1。
    static int positionValueOffset(const BufferGeometry& geometry);

    // 根据BufferGeometry局部Position数据计算局部轴对齐包围盒。
    static AxisAlignedBoundingBox geometryBounds(const BufferGeometry& geometry);

    // 为BRepViewerWidget建立默认环境光、主光和补光。
    static bool createDefaultBRepLighting(LightManager& lightManager);

    // 将MyBRep可逆仿射矩阵分解为MyOpenGL RenderItem支持的Translation-Rotation-Scale；含Shear时返回false。
    static bool decomposeItemTransform(
        const MyMath::Matrix4& localToWorld,
        QVector3D& position,
        QQuaternion& rotation,
        QVector3D& scale);

    // 查找可复用的Topology_Solid共享局部Geometry资源。
    BRepSolidGeometryResourceId findSolidGeometryResource(const Topology_Solid& solid) const;

    // 获取或创建Topology_Solid共享局部Geometry资源。
    BRepSolidGeometryResourceId acquireSolidGeometryResource(const Topology_Solid& solid,const QString& name);

    // 返回指定共享Solid Geometry资源记录，不存在时返回空指针。
    const BRepSolidGeometryResource* solidGeometryResource(BRepSolidGeometryResourceId id) const;

    // 使用共享Solid Geometry资源创建独立RenderItem和Material。
    BRepDisplayId attachSharedSolidDisplay(
        BRepSolidGeometryResourceId geometryResourceId,
        const MyMath::Matrix4& localToWorld,
        const QString& name,
        const BRepDisplayStyle& style);

    // 删除指定Surface/Boundary Geometry资源；已不存在的资源视为成功。
    bool removeGeometryResources(ResourceId surfaceGeometryId,ResourceId wireframeGeometryId);

    // 清空全部当前未再被Display引用的共享Solid Geometry缓存；调用前必须先清空Display。
    bool clearSolidGeometryResources();

    BRepDisplayId attachWireframe(BufferGeometry* geometry, const QString& name, const BRepDisplayStyle& style);
    BRepDisplayId attachSurfaceDisplay(BufferGeometry* surfaceGeometry, BufferGeometry* wireframeGeometry, const QString& name,
                                       const BRepDisplayStyle& style);
    bool removeDisplayResources(BRepDisplayObject& object);
    BRepDisplayId allocateDisplayId();
    BRepSolidGeometryResourceId allocateSolidGeometryResourceId();

private:
    typedef std::map<BRepDisplayId, BRepDisplayObject> DisplayMap;
    typedef std::map<BRepSolidGeometryResourceId, BRepSolidGeometryResource> SolidGeometryResourceMap;

    DisplayMap m_displays;
    SolidGeometryResourceMap m_solidGeometryResources;
    BRepSolidBuildOptions m_buildOptions; // 当前Viewer全部B-Rep显示统一使用的离散构建参数。
    BRepDisplayId m_nextDisplayId;
    BRepSolidGeometryResourceId m_nextSolidGeometryResourceId;
};

}
}

#endif // MYBREPOPENGL_VIEWER_BREPVIEWERWIDGET_H
