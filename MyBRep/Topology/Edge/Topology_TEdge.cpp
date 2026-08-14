#include "Topology_TEdge.h"

#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"

namespace
{

// 判断数值是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

// 判断数值是否为有限非负数。
bool isFiniteNonNegative(double value)
{
    return isFiniteValue(value) && value >= 0.0;
}

}

namespace MyBRep
{

Topology_TEdge::Topology_TEdge(const Topology_Vertex& startVertex,
                               const Topology_Vertex& endVertex,
                               const Foundation::RefPtr<const Geometry_Curve>& geometry,
                               double firstParameter,
                               double lastParameter,
                               double connectionTolerance)
    : m_startVertex(startVertex)
    , m_endVertex(endVertex)
    , m_geometry(geometry)
    , m_firstParameter(firstParameter)
    , m_lastParameter(lastParameter)
{
    MYBREP_ASSERT_MESSAGE(startVertex.isValid(),
                          "Topology_TEdge start vertex must be valid.");
    MYBREP_ASSERT_MESSAGE(endVertex.isValid(),
                          "Topology_TEdge end vertex must be valid.");
    MYBREP_ASSERT_MESSAGE(geometry,
                          "Topology_TEdge geometry must be non-null.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(firstParameter) && isFiniteValue(lastParameter),
                          "Topology_TEdge curve parameters must be finite.");
    MYBREP_ASSERT_MESSAGE(firstParameter < lastParameter,
                          "Topology_TEdge requires firstParameter < lastParameter.");
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(connectionTolerance),
                          "Topology_TEdge connection tolerance must be finite and non-negative.");
    MYBREP_ASSERT_MESSAGE(geometry->isParameterInDomain(firstParameter),
                          "Topology_TEdge firstParameter is outside the geometry natural parameter domain.");
    MYBREP_ASSERT_MESSAGE(geometry->isParameterInDomain(lastParameter),
                          "Topology_TEdge lastParameter is outside the geometry natural parameter domain.");

    if (geometry->isPeriodic())
    {
        MYBREP_ASSERT_MESSAGE(lastParameter - firstParameter <= geometry->period(),
                              "Topology_TEdge parameter interval must not exceed one geometry period.");
    }

    const MyMath::Vector3 geometryStart = geometry->pointAt(firstParameter);
    const MyMath::Vector3 geometryEnd = geometry->pointAt(lastParameter);

    MYBREP_ASSERT_MESSAGE(startVertex.point().isEqualTo(geometryStart, connectionTolerance),
                          "Topology_TEdge start vertex must match geometry at firstParameter.");
    MYBREP_ASSERT_MESSAGE(endVertex.point().isEqualTo(geometryEnd, connectionTolerance),
                          "Topology_TEdge end vertex must match geometry at lastParameter.");
}

/// 拓扑数据

const Topology_Vertex& Topology_TEdge::startVertex() const
{
    return m_startVertex;
}

const Topology_Vertex& Topology_TEdge::endVertex() const
{
    return m_endVertex;
}

/// 三维主曲线表示

const Geometry_Curve& Topology_TEdge::geometry() const
{
    return *m_geometry;
}

const Foundation::RefPtr<const Geometry_Curve>& Topology_TEdge::geometryResource() const
{
    return m_geometry;
}

double Topology_TEdge::firstParameter() const
{
    return m_firstParameter;
}

double Topology_TEdge::lastParameter() const
{
    return m_lastParameter;
}

/// Curve-on-Surface表示

std::size_t Topology_TEdge::curveOnSurfaceCount() const
{
    return m_curveOnSurfaces.size();
}

bool Topology_TEdge::hasCurveOnSurface(const Geometry_Surface& surface) const
{
    return findCurveOnSurface(surface) < m_curveOnSurfaces.size();
}

bool Topology_TEdge::isSeamOnSurface(const Geometry_Surface& surface) const
{
    const CurveOnSurfaceRepresentation& representation = curveOnSurface(surface);
    return static_cast<bool>(representation.secondCurve);
}

/// Curve-on-Surface内部访问

std::size_t Topology_TEdge::findCurveOnSurface(const Geometry_Surface& surface) const
{
    for (std::size_t index = 0; index < m_curveOnSurfaces.size(); ++index)
    {
        if (m_curveOnSurfaces[index].surface.get() == &surface)
        {
            return index;
        }
    }

    return m_curveOnSurfaces.size();
}

const Topology_TEdge::CurveOnSurfaceRepresentation&
Topology_TEdge::curveOnSurface(const Geometry_Surface& surface) const
{
    const std::size_t index = findCurveOnSurface(surface);

    MYBREP_ASSERT_MESSAGE(index < m_curveOnSurfaces.size(),
                          "Topology_TEdge has no Curve-on-Surface representation for the requested surface.");

    return m_curveOnSurfaces[index];
}

Topology_TEdge::CurveOnSurfaceRepresentation&
Topology_TEdge::curveOnSurface(const Geometry_Surface& surface)
{
    const std::size_t index = findCurveOnSurface(surface);

    MYBREP_ASSERT_MESSAGE(index < m_curveOnSurfaces.size(),
                          "Topology_TEdge has no Curve-on-Surface representation for the requested surface.");

    return m_curveOnSurfaces[index];
}

void Topology_TEdge::addCurveOnSurface(
    const Foundation::RefPtr<const Geometry_Surface>& surface,
    const Foundation::RefPtr<const Geometry_Curve2D>& curve,
    double curveFirstParameter,
    double curveLastParameter)
{
    MYBREP_ASSERT_MESSAGE(surface,
                          "Topology_TEdge Curve-on-Surface requires a non-null surface.");
    MYBREP_ASSERT_MESSAGE(curve,
                          "Topology_TEdge Curve-on-Surface requires a non-null 2D curve.");
    MYBREP_ASSERT_MESSAGE(!hasCurveOnSurface(*surface),
                          "Topology_TEdge already has a Curve-on-Surface representation for this surface.");

    CurveOnSurfaceRepresentation representation;
    representation.surface = surface;
    representation.firstCurve = curve;
    representation.firstCurveFirstParameter = curveFirstParameter;
    representation.firstCurveLastParameter = curveLastParameter;
    representation.secondCurveFirstParameter = 0.0;
    representation.secondCurveLastParameter = 0.0;

    m_curveOnSurfaces.push_back(representation);
}

void Topology_TEdge::addCurveOnClosedSurface(
    const Foundation::RefPtr<const Geometry_Surface>& surface,
    const Foundation::RefPtr<const Geometry_Curve2D>& firstCurve,
    double firstCurveFirstParameter,
    double firstCurveLastParameter,
    const Foundation::RefPtr<const Geometry_Curve2D>& secondCurve,
    double secondCurveFirstParameter,
    double secondCurveLastParameter)
{
    MYBREP_ASSERT_MESSAGE(surface,
                          "Topology_TEdge seam representation requires a non-null surface.");
    MYBREP_ASSERT_MESSAGE(firstCurve && secondCurve,
                          "Topology_TEdge seam representation requires two non-null 2D curves.");
    MYBREP_ASSERT_MESSAGE(!hasCurveOnSurface(*surface),
                          "Topology_TEdge already has a Curve-on-Surface representation for this surface.");

    CurveOnSurfaceRepresentation representation;
    representation.surface = surface;
    representation.firstCurve = firstCurve;
    representation.firstCurveFirstParameter = firstCurveFirstParameter;
    representation.firstCurveLastParameter = firstCurveLastParameter;
    representation.secondCurve = secondCurve;
    representation.secondCurveFirstParameter = secondCurveFirstParameter;
    representation.secondCurveLastParameter = secondCurveLastParameter;

    m_curveOnSurfaces.push_back(representation);
}

}
