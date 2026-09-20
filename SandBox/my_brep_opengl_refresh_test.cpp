#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <QApplication>
#include <QVector3D>
#include <QVector4D>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Instance/Face.h"
#include "MyBRep/Instance/Wire.h"
#include "MyBRep/Modeling/Edge/EdgeModeling.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"
#include "MyBRep/Tool/Collector/TopologyCollector.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"

#include "MyBRepOpenGL/Display/BRepDisplayManager.h"
#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

#include "MyOpenGL/Core/ResourceManager.h"
#include "MyOpenGL/Item/RenderItem.h"
#include "MyOpenGL/Item/RenderPart.h"
#include "MyOpenGL/Material/Material.h"
#include "MyOpenGL/Material/MaterialManager.h"
#include "MyOpenGL/Resource/Geometry.h"

namespace
{

const float FloatTolerance = 1.0e-5f;

class TestContext
{
public:
    TestContext() : m_passed(0), m_failed(0){}

    void expect(bool condition, const std::string& name)
    {
        if (condition)
        {
            ++m_passed;
            std::cout << "[PASS] " << name << std::endl;
        }
        else
        {
            ++m_failed;
            std::cout << "[FAIL] " << name << std::endl;
        }
    }

    int passed() const{return m_passed;}
    int failed() const{return m_failed;}

private:
    int m_passed;
    int m_failed;
};

struct PartSnapshot
{
    std::vector<RenderPartId> ids;
    std::vector<const Geometry*> geometries;
};

bool nearFloat(float first, float second)
{
    return std::fabs(first - second) <= FloatTolerance;
}

bool sameVector3(const QVector3D& first, const QVector3D& second)
{
    return nearFloat(first.x(), second.x()) &&
           nearFloat(first.y(), second.y()) &&
           nearFloat(first.z(), second.z());
}

bool sameColor(const QVector4D& first, const QVector4D& second)
{
    return nearFloat(first.x(), second.x()) &&
           nearFloat(first.y(), second.y()) &&
           nearFloat(first.z(), second.z()) &&
           nearFloat(first.w(), second.w());
}

PartSnapshot snapshotParts(const RenderItem& item)
{
    PartSnapshot result;
    result.ids.reserve(static_cast<std::size_t>(item.partCount()));
    result.geometries.reserve(static_cast<std::size_t>(item.partCount()));

    for (int index = 0; index < item.partCount(); ++index)
    {
        const RenderPart* part = item.partAt(index);
        if (part == 0) continue;

        result.ids.push_back(part->id());
        result.geometries.push_back(part->geometry());
    }

    return result;
}

bool samePartSnapshot(const RenderItem& item, const PartSnapshot& snapshot)
{
    if (item.partCount() != static_cast<int>(snapshot.ids.size())) return false;

    for (int index = 0; index < item.partCount(); ++index)
    {
        const RenderPart* part = item.partAt(index);
        if (part == 0) return false;

        const std::size_t vectorIndex = static_cast<std::size_t>(index);

        if (part->id() != snapshot.ids[vectorIndex]) return false;
        if (part->geometry() != snapshot.geometries[vectorIndex]) return false;
    }

    return true;
}

bool allLinePartsHaveWidth(const RenderItem& item, float width)
{
    bool foundLine = false;

    for (int index = 0; index < item.partCount(); ++index)
    {
        const RenderPart* part = item.partAt(index);
        if (part == 0 || part->geometry() == 0) return false;

        const RenderType renderType = part->geometry()->renderType();

        if (renderType == RenderType::Lines || renderType == RenderType::LineStrip)
        {
            foundLine = true;
            if (!nearFloat(part->lineWidth(), width)) return false;
        }
    }

    return foundLine;
}

bool partsUseExpectedMaterials(const RenderItem& item,
                               const Material* surfaceMaterial,
                               const Material* wireMaterial)
{
    bool foundSurface = false;
    bool foundWire = false;

    for (int index = 0; index < item.partCount(); ++index)
    {
        const RenderPart* part = item.partAt(index);
        if (part == 0 || part->geometry() == 0) return false;

        const RenderType renderType = part->geometry()->renderType();

        if (renderType == RenderType::Triangles)
        {
            foundSurface = true;
            if (part->material() != surfaceMaterial) return false;
        }
        else if (renderType == RenderType::Lines || renderType == RenderType::LineStrip)
        {
            foundWire = true;
            if (part->material() != wireMaterial) return false;
        }
    }

    return foundSurface && foundWire;
}

bool resourcesAreReleased(MyBRep::Display::BRepViewerWidget& viewer,
                          const std::vector<MyBRep::TopologyId>& topologyIds,
                          const std::vector<ResourceId>& resourceIds)
{
    if (topologyIds.size() != resourceIds.size()) return false;

    for (std::size_t index = 0; index < topologyIds.size(); ++index)
    {
        if (viewer.displayManager().containsTopology(topologyIds[index])) return false;
        if (viewer.resourceManager().contains(resourceIds[index])) return false;
    }

    return true;
}

void testWireRefresh(TestContext& context)
{
    using namespace MyBRep;
    using namespace MyBRep::Display;

    std::cout << std::endl << "=== Wire refresh ===" << std::endl;

    BRepViewerWidget viewer;

    Topology_Edge sharedEdge;
    TopologyId oldSecondEdgeId = InvalidTopologyId;
    Wire wire;

    {
        const Topology_Vertex vertexA(MyMath::Vector3(0.0, 0.0, 0.0));
        const Topology_Vertex vertexB(MyMath::Vector3(10.0, 0.0, 0.0));
        const Topology_Vertex vertexC(MyMath::Vector3(10.0, 8.0, 0.0));

        sharedEdge = Modeling::createLine(vertexA, vertexB);
        const Topology_Edge oldSecondEdge = Modeling::createLine(vertexB, vertexC);

        std::vector<Topology_Edge> edges;
        edges.push_back(sharedEdge);
        edges.push_back(oldSecondEdge);

        wire = Wire(Modeling::createWire(edges));
        oldSecondEdgeId = oldSecondEdge.id();
    }

    context.expect(wire.isValid(), "Wire fixture is valid");

    const InstanceId instanceId = wire.id();

    BRepDisplayStyle initialStyle;
    initialStyle.wireColor = QVector4D(0.15f, 0.25f, 0.75f, 1.0f);
    initialStyle.wireWidth = 1.25f;

    const BRepDisplayId displayId = viewer.addWireframe(wire, "RefreshWire", initialStyle);
    context.expect(displayId != InvalidBRepDisplayId, "Add Wire display");

    const BRepDisplayObject initialDisplay = viewer.display(displayId);
    context.expect(initialDisplay.itemId != InvalidRenderItemId, "Wire display has RenderItem");
    context.expect(initialDisplay.surfaceMaterialId == InvalidMaterialId, "Wire display has no surface Material");
    context.expect(initialDisplay.wireframeMaterialId != InvalidMaterialId, "Wire display has wireframe Material");
    context.expect(viewer.displayManager().itemId(instanceId) == initialDisplay.itemId,
                   "Wire InstanceId resolves initial RenderItemId");

    RenderItem* item = viewer.itemManager().get(initialDisplay.itemId);
    context.expect(item != 0, "Wire RenderItem exists");
    if (item == 0) return;

    context.expect(item->partCount() == 2, "Wire initially owns two RenderParts");

    const ResourceId sharedResourceId = viewer.displayManager().resourceId(sharedEdge.id());
    const ResourceId oldSecondResourceId = viewer.displayManager().resourceId(oldSecondEdgeId);

    context.expect(sharedResourceId != InvalidResourceId, "Shared Edge has Geometry Resource");
    context.expect(oldSecondResourceId != InvalidResourceId, "Old second Edge has Geometry Resource");
    context.expect(viewer.displayManager().topologyCount() == 2, "Wire display registers two Edge resources");

    const PartSnapshot initialParts = snapshotParts(*item);
    const QVector3D initialPosition = item->transform().position();

    BRepDisplayStyle refreshedStyle = initialStyle;
    refreshedStyle.wireColor = QVector4D(0.85f, 0.20f, 0.12f, 1.0f);
    refreshedStyle.wireWidth = 3.5f;

    context.expect(viewer.refreshDisplay(wire, refreshedStyle), "Display refresh succeeds");

    const BRepDisplayObject afterDisplayRefresh = viewer.display(displayId);
    context.expect(afterDisplayRefresh.itemId == initialDisplay.itemId,
                   "Display refresh preserves RenderItemId");
    context.expect(afterDisplayRefresh.wireframeMaterialId == initialDisplay.wireframeMaterialId,
                   "Display refresh preserves MaterialId");
    context.expect(samePartSnapshot(*item, initialParts),
                   "Display refresh preserves RenderPart identity and Geometry");
    context.expect(sameVector3(item->transform().position(), initialPosition),
                   "Display refresh preserves placement");

    const Material* wireMaterial = viewer.materialManager().get(initialDisplay.wireframeMaterialId);
    context.expect(wireMaterial != 0, "Wire Material remains available");
    if (wireMaterial != 0)
    {
        context.expect(sameColor(wireMaterial->color(), refreshedStyle.wireColor),
                       "Display refresh updates wire color");
        context.expect(!wireMaterial->lightingEnabled(),
                       "Wire Material remains unlit");
    }

    context.expect(allLinePartsHaveWidth(*item, refreshedStyle.wireWidth),
                   "Display refresh updates all line widths");
    context.expect(viewer.displayManager().resourceId(sharedEdge.id()) == sharedResourceId &&
                   viewer.displayManager().resourceId(oldSecondEdgeId) == oldSecondResourceId,
                   "Display refresh preserves Geometry Resources");

    const MyMath::Matrix4 placement =
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(3.0, 4.0, 5.0));

    context.expect(wire.setLocalToWorld(placement), "Wire accepts translated placement");
    context.expect(sameVector3(item->transform().position(), initialPosition),
                   "Instance placement change does not implicitly change RenderItem");
    context.expect(viewer.refreshPlacement(wire), "Placement refresh succeeds");
    context.expect(sameVector3(item->transform().position(), QVector3D(3.0f, 4.0f, 5.0f)),
                   "Placement refresh updates RenderItem position");
    context.expect(samePartSnapshot(*item, initialParts),
                   "Placement refresh preserves RenderParts and Geometry");
    context.expect(viewer.displayManager().resourceId(sharedEdge.id()) == sharedResourceId &&
                   viewer.displayManager().resourceId(oldSecondEdgeId) == oldSecondResourceId,
                   "Placement refresh preserves Geometry Resources");

    Topology_Edge newSecondEdge;
    {
        const Topology_Vertex vertexD(MyMath::Vector3(10.0, -6.0, 0.0));
        newSecondEdge = Modeling::createLine(sharedEdge.endVertex(), vertexD);

        std::vector<Topology_Edge> edges;
        edges.push_back(sharedEdge);
        edges.push_back(newSecondEdge);

        const Topology_Wire newTopology = Modeling::createWire(edges);
        const MyMath::Matrix4 currentPlacement = wire.localToWorld();
        wire = Wire(newTopology, currentPlacement);
    }

    context.expect(wire.id() == instanceId, "Wire assignment preserves InstanceId");
    context.expect(!viewer.displayManager().containsTopology(newSecondEdge.id()),
                   "New Edge has no display resource before topology refresh");
    context.expect(viewer.displayManager().containsTopology(oldSecondEdgeId),
                   "Old Edge resource remains before topology refresh");

    context.expect(viewer.refreshTopology(wire), "Topology refresh succeeds");

    const BRepDisplayObject afterTopologyRefresh = viewer.display(displayId);
    context.expect(afterTopologyRefresh.itemId == initialDisplay.itemId,
                   "Topology refresh preserves RenderItemId");
    context.expect(afterTopologyRefresh.wireframeMaterialId == initialDisplay.wireframeMaterialId,
                   "Topology refresh preserves wire MaterialId");

    item = viewer.itemManager().get(initialDisplay.itemId);
    context.expect(item != 0, "RenderItem still exists after topology refresh");
    if (item == 0) return;

    context.expect(item->partCount() == 2, "Topology refresh rebuilds two Wire RenderParts");
    context.expect(sameVector3(item->transform().position(), QVector3D(3.0f, 4.0f, 5.0f)),
                   "Topology refresh preserves placement");
    context.expect(viewer.displayManager().resourceId(sharedEdge.id()) == sharedResourceId,
                   "Topology refresh reuses unchanged Edge Geometry Resource");

    const ResourceId newSecondResourceId = viewer.displayManager().resourceId(newSecondEdge.id());
    context.expect(newSecondResourceId != InvalidResourceId,
                   "Topology refresh creates Geometry Resource for new Edge");
    context.expect(!viewer.displayManager().containsTopology(oldSecondEdgeId),
                   "Topology refresh removes obsolete Edge binding");
    context.expect(!viewer.resourceManager().contains(oldSecondResourceId),
                   "Topology refresh removes obsolete Edge Geometry Resource");
    context.expect(viewer.displayManager().topologyCount() == 2,
                   "Wire topology refresh leaves exactly two Edge resources");
    context.expect(allLinePartsHaveWidth(*item, refreshedStyle.wireWidth),
                   "Topology refresh preserves wire width");

    wireMaterial = viewer.materialManager().get(initialDisplay.wireframeMaterialId);
    context.expect(wireMaterial != 0 &&
                   sameColor(wireMaterial->color(), refreshedStyle.wireColor),
                   "Topology refresh preserves wire Material state");

    Wire undisplayed = Modeling::makeRectangle(2.0, 2.0);
    context.expect(!viewer.refreshDisplay(undisplayed, refreshedStyle),
                   "Display refresh rejects an undisplayed Instance");
    context.expect(!viewer.refreshPlacement(undisplayed),
                   "Placement refresh rejects an undisplayed Instance");
    context.expect(!viewer.refreshTopology(undisplayed),
                   "Topology refresh rejects an undisplayed Instance");

    Wire invalid;
    context.expect(!viewer.refreshDisplay(invalid, refreshedStyle),
                   "Display refresh rejects invalid Instance");
    context.expect(!viewer.refreshPlacement(invalid),
                   "Placement refresh rejects invalid Instance");
    context.expect(!viewer.refreshTopology(invalid),
                   "Topology refresh rejects invalid Instance");
}

void testFaceTopologyRefresh(TestContext& context)
{
    using namespace MyBRep;
    using namespace MyBRep::Display;

    std::cout << std::endl << "=== Face topology refresh ===" << std::endl;

    BRepViewerWidget viewer;
    Face face;

    {
        const Topology_Wire oldWire = Modeling::createRectangle(8.0, 5.0);
        const Topology_Face oldFace = Modeling::createPlanarFace(oldWire);
        face = Face(oldFace, MyMath::Matrix4::fromTranslation(MyMath::Vector3(-2.0, 1.5, 4.0)));
    }

    context.expect(face.isValid(), "Face fixture is valid");

    const InstanceId instanceId = face.id();

    BRepDisplayStyle style;
    style.surfaceColor = QVector4D(0.25f, 0.65f, 0.82f, 1.0f);
    style.wireColor = QVector4D(0.08f, 0.09f, 0.12f, 1.0f);
    style.surfaceLightingEnabled = true;
    style.wireWidth = 2.25f;

    const BRepDisplayId displayId = viewer.addFace(face, "RefreshFace", style);
    context.expect(displayId != InvalidBRepDisplayId, "Add Face display");

    const BRepDisplayObject initialDisplay = viewer.display(displayId);
    context.expect(initialDisplay.itemId != InvalidRenderItemId, "Face display has RenderItem");
    context.expect(initialDisplay.surfaceMaterialId != InvalidMaterialId, "Face display has surface Material");
    context.expect(initialDisplay.wireframeMaterialId != InvalidMaterialId, "Face display has wireframe Material");
    context.expect(viewer.displayManager().itemId(instanceId) == initialDisplay.itemId,
                   "Face InstanceId resolves initial RenderItemId");

    RenderItem* item = viewer.itemManager().get(initialDisplay.itemId);
    context.expect(item != 0, "Face RenderItem exists");
    if (item == 0) return;

    context.expect(item->partCount() == 5, "Planar rectangle Face has one Face Part and four Edge Parts");

    const Material* surfaceMaterial = viewer.materialManager().get(initialDisplay.surfaceMaterialId);
    const Material* wireMaterial = viewer.materialManager().get(initialDisplay.wireframeMaterialId);

    context.expect(surfaceMaterial != 0 && wireMaterial != 0, "Face Materials are available");
    context.expect(partsUseExpectedMaterials(*item, surfaceMaterial, wireMaterial),
                   "Initial Face Parts use surface/wire Materials by render type");

    std::vector<TopologyId> oldTopologyIds;
    std::vector<ResourceId> oldResourceIds;

    {
        const Tool::TopologyCollection oldCollection = Tool::TopologyCollector::collect(face.topology());

        oldTopologyIds.reserve(oldCollection.faces.size() + oldCollection.edges.size());
        oldResourceIds.reserve(oldCollection.faces.size() + oldCollection.edges.size());

        for (std::size_t index = 0; index < oldCollection.faces.size(); ++index)
        {
            oldTopologyIds.push_back(oldCollection.faces[index].id());
            oldResourceIds.push_back(viewer.displayManager().resourceId(oldCollection.faces[index].id()));
        }

        for (std::size_t index = 0; index < oldCollection.edges.size(); ++index)
        {
            oldTopologyIds.push_back(oldCollection.edges[index].id());
            oldResourceIds.push_back(viewer.displayManager().resourceId(oldCollection.edges[index].id()));
        }
    }

    bool oldResourcesValid = oldTopologyIds.size() == 5 && oldResourceIds.size() == 5;
    for (std::size_t index = 0; index < oldResourceIds.size(); ++index)
    {
        oldResourcesValid = oldResourcesValid && oldResourceIds[index] != InvalidResourceId;
    }
    context.expect(oldResourcesValid, "Initial Face and Edge Geometry Resources are registered");

    const QVector3D oldPosition = item->transform().position();
    const MaterialId surfaceMaterialId = initialDisplay.surfaceMaterialId;
    const MaterialId wireMaterialId = initialDisplay.wireframeMaterialId;
    const RenderItemId itemId = initialDisplay.itemId;

    {
        const Topology_Wire newWire =
            Modeling::createRectangle(MyMath::Vector3(1.0, -0.5, 0.0), 6.0, 3.0);
        const Topology_Face newFace = Modeling::createPlanarFace(newWire);
        const MyMath::Matrix4 placement = face.localToWorld();
        face = Face(newFace, placement);
    }

    context.expect(face.id() == instanceId, "Face assignment preserves InstanceId");
    context.expect(viewer.refreshTopology(face), "Face topology refresh succeeds");

    const BRepDisplayObject refreshedDisplay = viewer.display(displayId);
    context.expect(refreshedDisplay.itemId == itemId, "Face topology refresh preserves RenderItemId");
    context.expect(refreshedDisplay.surfaceMaterialId == surfaceMaterialId,
                   "Face topology refresh preserves surface MaterialId");
    context.expect(refreshedDisplay.wireframeMaterialId == wireMaterialId,
                   "Face topology refresh preserves wire MaterialId");

    item = viewer.itemManager().get(itemId);
    context.expect(item != 0, "Face RenderItem remains after topology refresh");
    if (item == 0) return;

    context.expect(item->partCount() == 5, "Refreshed rectangle Face still has five RenderParts");
    context.expect(sameVector3(item->transform().position(), oldPosition),
                   "Face topology refresh preserves placement");
    context.expect(allLinePartsHaveWidth(*item, style.wireWidth),
                   "Face topology refresh preserves wire width");

    surfaceMaterial = viewer.materialManager().get(surfaceMaterialId);
    wireMaterial = viewer.materialManager().get(wireMaterialId);

    context.expect(surfaceMaterial != 0 && wireMaterial != 0,
                   "Face Materials remain after topology refresh");
    context.expect(partsUseExpectedMaterials(*item, surfaceMaterial, wireMaterial),
                   "Refreshed Face Parts keep original Materials");

    const Tool::TopologyCollection newCollection = Tool::TopologyCollector::collect(face.topology());
    bool newResourcesValid = newCollection.faces.size() == 1 && newCollection.edges.size() == 4;

    for (std::size_t index = 0; index < newCollection.faces.size(); ++index)
    {
        newResourcesValid =
            newResourcesValid &&
            viewer.displayManager().resourceId(newCollection.faces[index].id()) != InvalidResourceId;
    }

    for (std::size_t index = 0; index < newCollection.edges.size(); ++index)
    {
        newResourcesValid =
            newResourcesValid &&
            viewer.displayManager().resourceId(newCollection.edges[index].id()) != InvalidResourceId;
    }

    context.expect(newResourcesValid, "Face topology refresh registers all new Face/Edge resources");

    context.expect(resourcesAreReleased(viewer, oldTopologyIds, oldResourceIds),
                   "Face topology refresh releases all obsolete Face/Edge resources in the same refresh");

    context.expect(viewer.displayManager().topologyCount() == 5,
                   "Face topology refresh leaves exactly one Face and four Edge resources");
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    TestContext context;

    testWireRefresh(context);
    testFaceTopologyRefresh(context);

    std::cout << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << std::endl;
    std::cout << "Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}