#ifndef RENDERPART_H
#define RENDERPART_H

#include "AxisAlignedBoundingBox.h"

#include <QPointF>
#include <QVector2D>
#include <QVector3D>

#include <cstdint>
#include <vector>

class Geometry;
class Light;
class Material;
class RenderItem;
class ItemManager;
class Renderer;
struct RenderContext;
struct RenderState;

/// RenderPart 唯一标识类型，由 RenderItem 统一分配。
typedef std::uint64_t RenderPartId;

/// 无效 RenderPart ID。
const RenderPartId InvalidRenderPartId = static_cast<RenderPartId>(0);

/// RenderPart 对 RenderItem Depth 状态的覆盖方式。
enum class RenderPartStateMode
{
    Inherit,
    Enabled,
    Disabled
};

/// RenderItem 内具有稳定身份的最小可绘制单位。
/// RenderPart 不拥有其引用的 Geometry 和 Material。
class RenderPart
{
public:
    /// Identity

    RenderPartId id() const;

    /// Render

    /// 使用当前 Part 自己的 Anchor 和显示规则绘制 Geometry。
    virtual bool draw(Renderer& renderer,
                      const RenderItem& item,
                      const RenderContext& context,
                      const std::vector<const Light*>& lights) const;

    /// 返回当前 Part 按最终绘制规则是否属于透明内容。
    /// 标准 Triangle Wireframe 使用 Item edgeColor Alpha 判断；
    /// 其余绘制使用 Part Material 或 Item Material 的 BlendMode 判断。
    bool isTransparent(const RenderItem& item) const;

    /// Geometry

    const Geometry* geometry() const{return m_geometry;}
    void setGeometry(const Geometry* geometry){m_geometry = geometry;}

    /// Material

    const Material* material() const{return m_material;}
    void setMaterial(const Material* material){m_material = material;}

    /// Bounds

    bool hasLocalBounds() const;
    const AxisAlignedBoundingBox& localBounds() const;
    void setLocalBounds(const AxisAlignedBoundingBox& bounds);
    void clearLocalBounds();

    /// Anchor

    const QVector3D& anchor3D() const{return m_anchor3D;}
    void setAnchor3D(const QVector3D& anchor){m_anchor3D = anchor;}

    const QVector2D& anchor2D() const{return m_anchor2D;}
    void setAnchor2D(const QVector2D& anchor){m_anchor2D = anchor;}

    const QPointF& anchorPixel() const{return m_anchorPixel;}
    void setAnchorPixel(const QPointF& anchor){m_anchorPixel = anchor;}

    /// Display Space

    bool followCamera() const{return m_followCamera;}
    void setFollowCamera(bool enabled){m_followCamera = enabled;}

    bool pixelScale() const{return m_pixelScale;}
    void setPixelScale(bool enabled){m_pixelScale = enabled;}

    bool isStandardModel() const{return !m_followCamera && !m_pixelScale;}

    /// Depth

    RenderPartStateMode depthTestMode() const{return m_depthTestMode;}
    void setDepthTestMode(RenderPartStateMode mode){m_depthTestMode = mode;}

    RenderPartStateMode depthWriteMode() const{return m_depthWriteMode;}
    void setDepthWriteMode(RenderPartStateMode mode){m_depthWriteMode = mode;}

    float lineWidth() const{return m_lineWidth;}
    void setLineWidth(float lineWidth){m_lineWidth = lineWidth;}


protected:
    friend class RenderItem;
    friend class ItemManager;
    /// RenderItem 内部接口。

    explicit RenderPart(RenderPartId id);
    virtual ~RenderPart();

    /// 使用当前 Part 自己的参数构造 RenderState。
    bool buildRenderState(const RenderItem& item,
                          const RenderContext& context,
                          RenderState& state) const;

    /// 使用指定 Item Local Space Anchor 构造 RenderState。
    bool buildRenderState(const RenderItem& item,
                          const RenderContext& context,
                          const QVector3D& anchor3D,
                          const QVector2D& anchor2D,
                          const QPointF& anchorPixel,
                          RenderState& state) const;

protected:
    RenderPartId m_id = InvalidRenderPartId;
    const Geometry* m_geometry = 0;
    const Material* m_material = 0;
    float m_lineWidth = 1.0f;
    AxisAlignedBoundingBox m_localBounds;
    QVector3D m_anchor3D = QVector3D(0.0f, 0.0f, 0.0f);
    QVector2D m_anchor2D = QVector2D(0.0f, 0.0f);
    QPointF m_anchorPixel = QPointF(0.0, 0.0);
    bool m_followCamera = false;
    bool m_pixelScale = false;
    RenderPartStateMode m_depthTestMode = RenderPartStateMode::Inherit;
    RenderPartStateMode m_depthWriteMode = RenderPartStateMode::Inherit;
};

#endif // RENDERPART_H