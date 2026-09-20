#ifndef MYBREPOPENGL_VIEWER_BREPVIEWERWIDGET_H
#define MYBREPOPENGL_VIEWER_BREPVIEWERWIDGET_H

#include <cstddef>
#include <map>
#include <vector>

#include <QString>

#include "MyMath/Matrix4.h"

#include "MyBRep/Instance/Edge.h"
#include "MyBRep/Instance/Face.h"
#include "MyBRep/Instance/Shell.h"
#include "MyBRep/Instance/Solid.h"
#include "MyBRep/Instance/Wire.h"
#include "MyBRep/Instance/Instance.h"
#include "MyBRep/Tool/Collector/TopologyCollector.h"

#include "MyBRepOpenGL/Builder/BRepEdgeBuilder.h"
#include "MyBRepOpenGL/Builder/BRepFaceBuilder.h"
#include "MyBRepOpenGL/Display/BRepDisplayManager.h"
#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyOpenGL/Viewer/OpenGLViewerWidget.h"

class AxisAlignedBoundingBox;
class BufferGeometry;
class LightManager;
class Material;
class QQuaternion;
class QVector3D;
class RenderItem;

namespace MyBRep
{
namespace Display
{

struct BRepViewerBuildOptions
{
    bool isValid() const { return face.isValid() && edge.isValid(); }

    BRepFaceBuildOptions face;
    BRepEdgeBuildOptions edge;
};

// 一个B-Rep Instance对应一个RenderItem。
// Face/Edge拓扑对应共享Geometry Resource；RenderPart由ItemManager拥有，RenderItem只负责组织。
class BRepViewerWidget : public OpenGLViewerWidget
{
public:
    explicit BRepViewerWidget(QWidget* parent = 0);
    ~BRepViewerWidget() override;

    /// Wireframe Instance

    BRepDisplayId addWireframe(const Edge& edge, const QString& name = "BRepEdge",
                               const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Wire& wire, const QString& name = "BRepWire",
                               const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Face& face, const QString& name = "BRepFaceWireframe",
                               const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Shell& shell, const QString& name = "BRepShellWireframe",
                               const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addWireframe(const Solid& solid, const QString& name = "BRepSolidWireframe",
                               const BRepDisplayStyle& style = BRepDisplayStyle());

    /// Surface + Edge Instance

    BRepDisplayId addFace(const Face& face, const QString& name = "BRepFace",
                          const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addShell(const Shell& shell, const QString& name = "BRepShell",
                           const BRepDisplayStyle& style = BRepDisplayStyle());
    BRepDisplayId addSolid(const Solid& solid, const QString& name = "BRepSolid",
                           const BRepDisplayStyle& style = BRepDisplayStyle());

    /// 全局离散参数

    const BRepViewerBuildOptions& buildOptions() const;
    bool setBuildOptions(const BRepViewerBuildOptions& options);

    /// 身份映射

    const BRepDisplayManager& displayManager() const;

    /// 显示对象管理

    bool containsDisplay(BRepDisplayId id) const;
    std::size_t displayCount() const;
    BRepDisplayObject display(BRepDisplayId id) const;

    bool removeDisplay(BRepDisplayId id);
    bool clearBRepDisplays();

    /// 刷新

    // 刷新指定Instance当前Display的颜色、线宽和光照等显示属性。
    // 不修改Topology、Geometry Resource或空间放置。
    bool refreshDisplay(const Instance& instance, const BRepDisplayStyle& style);

    // 将指定Instance当前空间放置同步到对应RenderItem。
    // 不修改Topology、Geometry Resource、RenderPart或Material。
    bool refreshPlacement(const Instance& instance);

    // 将指定Instance当前Topology同步到对应RenderItem。
    // 保持Instance、RenderItem、Material和空间放置不变，重新组织Face/Edge RenderPart。
    bool refreshTopology(const Instance& instance);

private:
    static int positionValueOffset(const BufferGeometry& geometry);
    static AxisAlignedBoundingBox geometryBounds(const BufferGeometry& geometry);
    static bool createDefaultBRepLighting(LightManager& lightManager);

    // 当前只支持Translation-Rotation-Scale，不支持Shear。
    static bool decomposeItemTransform(const MyMath::Matrix4& localToWorld,
                                       QVector3D& position,
                                       QQuaternion& rotation,
                                       QVector3D& scale);

    /// Topology Geometry Resource

    ResourceId acquireFaceResource(const Topology_Face& face, const QString& name);
    ResourceId acquireEdgeResource(const Topology_Edge& edge, const QString& name);

    // 回收只剩BRepDisplayManager自身一份RefPtr引用的Topology Geometry。
    bool clearUnusedTopologyResources();

    // 强制删除当前Viewer持有的全部Topology Geometry，用于Viewer整体销毁或离散参数重建。
    bool clearTopologyResources();

    bool removeTopologyResource(TopologyId topologyId, ResourceId resourceId);

    /// RenderPart组织

    bool attachFaceParts(RenderItem& item, const std::vector<Topology_Face>& faces,
                         const Material* material, const QString& name);
    bool attachEdgeParts(RenderItem& item, const std::vector<Topology_Edge>& edges,
                        const Material* material, float lineWidth, const QString& name);

    // 返回Item当前引用的全部RenderPart身份。
    static std::vector<RenderPartId> itemPartIds(const RenderItem& item);

    // 删除Item当前引用的指定RenderPart；Part由ItemManager拥有。
    bool removeItemParts(RenderItem& item, const std::vector<RenderPartId>& partIds);

    // 删除Item当前引用的全部RenderPart；Part由ItemManager拥有。
    bool removeItemParts(RenderItem& item);

    /// 统一Instance显示入口

    BRepDisplayId addInstance(InstanceId instanceId,
                              const Tool::TopologyCollection& topology,
                              const MyMath::Matrix4& localToWorld,
                              const QString& name,
                              const BRepDisplayStyle& style,
                              bool showFaces,
                              bool showEdges);

    /// 内部释放

    bool removeDisplayResources(BRepDisplayObject& object);

    BRepDisplayId allocateDisplayId();
    // 返回指定RenderItem对应的BRep Display；不存在时返回InvalidBRepDisplayId。
    BRepDisplayId displayIdByItem(RenderItemId itemId) const;

    // 按内部Display身份刷新显示属性。
    bool refreshDisplay(BRepDisplayId displayId, const BRepDisplayStyle& style);

    // 按Instance身份刷新对应RenderItem的空间放置。
    bool refreshPlacement(InstanceId instanceId, const MyMath::Matrix4& localToWorld);

    // 按内部Display身份刷新Topology组成。
    bool refreshTopology(BRepDisplayId displayId, const Tool::TopologyCollection& topology);

private:
    typedef std::map<BRepDisplayId, BRepDisplayObject> DisplayMap;

    DisplayMap m_displays;

    BRepViewerBuildOptions m_buildOptions;
    BRepDisplayManager m_displayManager;

    BRepDisplayId m_nextDisplayId;
};

}
}

#endif // MYBREPOPENGL_VIEWER_BREPVIEWERWIDGET_H