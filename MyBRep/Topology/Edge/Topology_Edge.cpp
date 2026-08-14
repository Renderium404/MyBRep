#include "Topology_Edge.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Topology_Edge::Topology_Edge()
{
}

Topology_Edge::Topology_Edge(const Topology_Vertex& startVertex,
                             const Topology_Vertex& endVertex,
                             const Foundation::RefPtr<const Geometry_Curve>& geometry,
                             double firstParameter,
                             double lastParameter,
                             double connectionTolerance)
    : Topology_Object(Foundation::RefPtr<Topology_TObject>(
                          new Topology_TEdge(startVertex,
                                             endVertex,
                                             geometry,
                                             firstParameter,
                                             lastParameter,
                                             connectionTolerance)),
                      Topology_Orientation::Forward)
{
}

Topology_Edge::Topology_Edge(const Foundation::RefPtr<Topology_TObject>& object,
                             Topology_Orientation orientation)
    : Topology_Object(object, orientation)
{
}

/// 拓扑数据

const Topology_Vertex& Topology_Edge::startVertex() const
{
    return isForward() ? tEdge().startVertex() : tEdge().endVertex();
}

const Topology_Vertex& Topology_Edge::endVertex() const
{
    return isForward() ? tEdge().endVertex() : tEdge().startVertex();
}

/// 三维主曲线表示

const Geometry_Curve& Topology_Edge::geometry() const
{
    return tEdge().geometry();
}

const Foundation::RefPtr<const Geometry_Curve>& Topology_Edge::geometryResource() const
{
    return tEdge().geometryResource();
}

double Topology_Edge::firstParameter() const
{
    return isForward() ? tEdge().firstParameter() : tEdge().lastParameter();
}

double Topology_Edge::lastParameter() const
{
    return isForward() ? tEdge().lastParameter() : tEdge().firstParameter();
}

double Topology_Edge::curveParameterAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot query an invalid Topology_Edge.");
    MYBREP_ASSERT_MESSAGE(parameter >= 0.0 && parameter <= 1.0,
                          "Topology_Edge normalized parameter must be in [0,1].");

    return firstParameter() +
           (lastParameter() - firstParameter()) * parameter;
}

MyMath::Vector3 Topology_Edge::pointAt(double parameter) const
{
    return geometry().pointAt(curveParameterAt(parameter));
}

MyMath::Vector3 Topology_Edge::tangentAt(double parameter) const
{
    const double curveParameter = curveParameterAt(parameter);
    MyMath::Vector3 tangent = geometry().tangentAt(curveParameter);

    if (lastParameter() < firstParameter())
    {
        tangent = tangent * -1.0;
    }

    return tangent;
}

/// Curve-on-Surface表示

std::size_t Topology_Edge::curveOnSurfaceCount() const
{
    return isValid() ? tEdge().curveOnSurfaceCount() : 0;
}

bool Topology_Edge::hasCurveOnSurface(const Geometry_Surface& surface) const
{
    return isValid() && tEdge().hasCurveOnSurface(surface);
}

bool Topology_Edge::isSeamOnSurface(const Geometry_Surface& surface) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot query an invalid Topology_Edge.");
    MYBREP_ASSERT_MESSAGE(hasCurveOnSurface(surface),
                          "Topology_Edge has no Curve-on-Surface representation for the requested surface.");

    return tEdge().isSeamOnSurface(surface);
}

const Geometry_Curve2D& Topology_Edge::curveOnSurface(const Geometry_Surface& surface) const
{
    return *curveOnSurfaceResource(surface);
}

const Foundation::RefPtr<const Geometry_Curve2D>&
Topology_Edge::curveOnSurfaceResource(const Geometry_Surface& surface) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot query an invalid Topology_Edge.");

    const Topology_TEdge::CurveOnSurfaceRepresentation& representation =
        tEdge().curveOnSurface(surface);

    if (representation.secondCurve && isReversed())
    {
        return representation.secondCurve;
    }

    return representation.firstCurve;
}

double Topology_Edge::curveOnSurfaceFirstParameter(const Geometry_Surface& surface) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot query an invalid Topology_Edge.");

    const Topology_TEdge::CurveOnSurfaceRepresentation& representation =
        tEdge().curveOnSurface(surface);

    if (representation.secondCurve && isReversed())
    {
        return representation.secondCurveLastParameter;
    }

    return isForward()
               ? representation.firstCurveFirstParameter
               : representation.firstCurveLastParameter;
}

double Topology_Edge::curveOnSurfaceLastParameter(const Geometry_Surface& surface) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot query an invalid Topology_Edge.");

    const Topology_TEdge::CurveOnSurfaceRepresentation& representation =
        tEdge().curveOnSurface(surface);

    if (representation.secondCurve && isReversed())
    {
        return representation.secondCurveFirstParameter;
    }

    return isForward()
               ? representation.firstCurveLastParameter
               : representation.firstCurveFirstParameter;
}

double Topology_Edge::curveOnSurfaceParameterAt(const Geometry_Surface& surface,
                                                double parameter) const
{
    MYBREP_ASSERT_MESSAGE(parameter >= 0.0 && parameter <= 1.0,
                          "Topology_Edge normalized parameter must be in [0,1].");

    return curveOnSurfaceFirstParameter(surface) +
           (curveOnSurfaceLastParameter(surface) -
            curveOnSurfaceFirstParameter(surface)) *
               parameter;
}

MyMath::Vector2 Topology_Edge::surfaceParameterAt(const Geometry_Surface& surface,
                                                  double parameter) const
{
    const Geometry_Curve2D& curve = curveOnSurface(surface);
    return curve.pointAt(curveOnSurfaceParameterAt(surface, parameter));
}

/// 拓扑创建

Topology_Edge Topology_Edge::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot reverse an invalid Topology_Edge.");

    return Topology_Edge(tObject(), reversedOrientation());
}

/// 内部访问

const Topology_TEdge& Topology_Edge::tEdge() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access an invalid Topology_Edge.");

    return *static_cast<const Topology_TEdge*>(tObject().get());
}

Topology_TEdge& Topology_Edge::mutableTEdge() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot modify an invalid Topology_Edge.");

    return *static_cast<Topology_TEdge*>(tObject().get());
}

}
