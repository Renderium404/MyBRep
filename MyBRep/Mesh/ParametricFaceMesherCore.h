#ifndef MYBREP_MESH_PARAMETRICFACEMESHERCORE_H
#define MYBREP_MESH_PARAMETRICFACEMESHERCORE_H

#include <map>
#include <set>
#include <vector>

#include "MyBRep/Mesh/FaceMesh.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// 通用正则参数曲面Face三角化参数。
// 该结构只描述Mesher算法约束，不规定具体SurfaceKind。
struct ParametricFaceMeshOptions
{
    ParametricFaceMeshOptions();

    // 判断当前参数是否满足通用参数曲面三角化约束。
    bool isValid() const;

    double boundaryChordTolerance;       // trimming P-Curve映射到三维Surface后允许的最大边界弦误差。
    double surfaceChordTolerance;        // Surface被三角形近似时允许的最大共享边三维弦高误差。
    double geometricTolerance;           // UV连接、合并、共线、相交和周期对齐使用的参数空间几何容差。
    int minimumBoundarySubdivisionDepth; // trimming Edge至少执行的二分深度。
    int maximumBoundarySubdivisionDepth; // trimming Edge自适应细分允许的最大二分深度。
    int maximumSurfaceSubdivisionRounds; // Surface共享边一致细分允许的最大迭代轮数。

private:
    enum
    {
        MaximumAllowedBoundarySubdivisionDepth = 20,
        MaximumAllowedSurfaceSubdivisionRounds = 20
    };
};

// 参数曲面奇点处理接口。
// 只有具体Surface Mesher明确理解自身奇点几何语义时才应提供实现；共享核心不猜测奇点的极限法向。
class ParametricFaceSingularityHandler
{
public:
    virtual ~ParametricFaceSingularityHandler()
    {
    }

    // 判断参数点是否属于当前Surface Mesher认可的参数奇点。
    virtual bool isSingular(const Geometry_Surface& surface, const MyMath::Vector2& parameter, double tolerance) const = 0;

    // 判断两个不同UV是否在同一个参数奇点处表示同一条拓扑连接。
    virtual bool parametersMeetAtSingularity(const Geometry_Surface& surface, const MyMath::Vector2& first,
                                             const MyMath::Vector2& second, double tolerance) const = 0;

    // 根据奇点参数及其相邻正则参数返回考虑Face方向后的单位极限法向。
    virtual bool normalAt(const Topology_Face& face, const MyMath::Vector2& singularParameter,
                          const MyMath::Vector2& regularApproachParameter, double tolerance, MyMath::Vector3& normal) const = 0;
};

// 通用参数曲面Mesher使用的参数域规则。
struct ParametricFaceMeshPolicy
{
    ParametricFaceMeshPolicy();

    bool periodicU;                                      // U方向是否按Surface周期展开。
    bool periodicV;                                      // V方向是否按Surface周期展开。
    bool rejectSingularParameters;                       // 是否拒绝dS/du × dS/dv退化的参数点。
    const ParametricFaceSingularityHandler* singularityHandler; // 非空时由具体Surface Mesher定义允许的奇点连接与极限法向。
};

// 为Extruded/Revolved等参数曲面提供共享三角化核心，支持U/V单周期、双周期以及由具体Mesher显式定义的边界参数奇点。
// 本类不判断SurfaceKind；具体Mesher必须先完成曲面类型和周期语义检查。
class ParametricFaceMesherCore
{
public:
    // 判断Face的Wire、P-Curve和参数域是否满足共享核心的基础前置条件。
    static bool canMesh(const Topology_Face& face, const ParametricFaceMeshPolicy& policy);

    // 执行P-Curve采样、UV even-odd三角化和三维共享边自适应细分。
    static FaceMesh mesh(const Topology_Face& face, const ParametricFaceMeshOptions& options, const ParametricFaceMeshPolicy& policy);

private:
    // UV参数域的环和对应的嵌套深度。
    // 我们简称UV环
    struct Ring2D
    {
        std::vector<MyMath::Vector2> points;
        int depth;
    };

    // 保存UV参数域中的一个有向三角形。
    struct Triangle2D
    {
        Triangle2D();
        Triangle2D(const MyMath::Vector2& firstValue, const MyMath::Vector2& secondValue, const MyMath::Vector2& thirdValue);

        MyMath::Vector2 first;
        MyMath::Vector2 second;
        MyMath::Vector2 third;
    };

    // 保存参数顶点数组中的一个索引三角形。
    struct IndexedTriangle
    {
        IndexedTriangle();
        IndexedTriangle(unsigned int firstValue, unsigned int secondValue, unsigned int thirdValue);

        unsigned int first;
        unsigned int second;
        unsigned int third;
    };

    // 表示无向索引边，索引自动按从小到大规范化。
    struct EdgeKey
    {
        EdgeKey();
        EdgeKey(unsigned int firstValue, unsigned int secondValue);

        // 提供std::set/std::map需要的严格弱序。
        bool operator<(const EdgeKey& other) const;

        unsigned int first;
        unsigned int second;
    };

    // 描述参数点相对于全部trimming Ring的分类结果。
    enum class TrimClassification
    {
        Outside,
        Boundary,
        Inside
    };

    // 返回标量绝对值。
    static double absoluteValue(double value);

    // 判断标量是否为有限值。
    static bool isFiniteValue(double value);

    // 返回最接近指定标量的整数值。
    static double nearestInteger(double value);

    // 返回二维点到有限二维线段的最短距离。
    static double pointSegmentDistance2D(const MyMath::Vector2& point, const MyMath::Vector2& start, const MyMath::Vector2& end);

    // 返回三维点到有限三维线段的最短距离。
    static double pointSegmentDistance3D(const MyMath::Vector3& point, const MyMath::Vector3& start, const MyMath::Vector3& end);

    // 判断两个二维点是否在指定容差内重合。
    static bool pointsEqual(const MyMath::Vector2& first, const MyMath::Vector2& second, double tolerance);

    // 返回三个二维点组成的有向面积二倍值。
    static double orientation(const MyMath::Vector2& first, const MyMath::Vector2& second, const MyMath::Vector2& third);

    // 返回闭合二维多边形的有向面积。
    static double signedArea(const std::vector<MyMath::Vector2>& polygon);

    // 判断二维点是否位于指定有限线段的容差带内。
    static bool pointOnSegment(const MyMath::Vector2& point, const MyMath::Vector2& first,
                               const MyMath::Vector2& second, double tolerance);

    // 判断两条二维有限线段是否在指定容差下相交。
    static bool segmentsIntersect(const MyMath::Vector2& firstStart, const MyMath::Vector2& firstEnd,
                                  const MyMath::Vector2& secondStart, const MyMath::Vector2& secondEnd, double tolerance);

    // 使用even-odd规则判断二维点是否位于单个Ring内部或边界上。
    static bool pointInRing(const MyMath::Vector2& point, const std::vector<MyMath::Vector2>& ring, double tolerance);

    // 使用全部Ring的even-odd规则分类指定参数点。
    static TrimClassification classifyTrim(const MyMath::Vector2& point, const std::vector<Ring2D>& rings, double tolerance);

    // 判断点是否严格位于逆时针三角形内部。
    static bool pointStrictlyInTriangle(const MyMath::Vector2& point, const MyMath::Vector2& first,
                                        const MyMath::Vector2& second, const MyMath::Vector2& third, double tolerance);

    // 删除二维点序列中相邻重复点以及首尾重复点。
    static void removeConsecutiveDuplicates(std::vector<MyMath::Vector2>& points, double tolerance);

    // 删除闭合点序列中可以安全移除的简单共线点。
    static void removeSimpleCollinearPoints(std::vector<MyMath::Vector2>& points, double tolerance);

    // 返回Edge规范化参数对应的Surface二维参数。
    static MyMath::Vector2 surfaceParameter(const Topology_Edge& edge, const Geometry_Surface& surface, double parameter);

    // 返回Surface二维参数对应的三维位置。
    static MyMath::Vector3 surfacePosition(const Geometry_Surface& surface, const MyMath::Vector2& parameter);

    // 判断一个trimming Edge参数区间映射到三维Surface后是否满足边界弦误差。
    static bool boundaryIntervalFlatEnough(const Topology_Edge& edge, const Geometry_Surface& surface,
                                           double firstParameter, double lastParameter, const MyMath::Vector2& firstUV,
                                           const MyMath::Vector2& lastUV, double chordTolerance);

    // 递归离散单个trimming Edge参数区间。
    static void appendAdaptiveBoundaryInterval(const Topology_Edge& edge, const Geometry_Surface& surface,
                                               double firstParameter, double lastParameter, const MyMath::Vector2& firstUV,
                                               const MyMath::Vector2& lastUV, int depth, const ParametricFaceMeshOptions& options,
                                               std::vector<MyMath::Vector2>& points);

    // 将单个Edge-use的P-Curve自适应采样为参数点序列。
    static bool sampleEdge(const Topology_Edge& edge, const Geometry_Surface& surface,
                           const ParametricFaceMeshOptions& options, std::vector<MyMath::Vector2>& points);

    // 返回二维参数指定轴的坐标值，axis为0时返回U，为1时返回V。
    static double parameterCoordinate(const MyMath::Vector2& parameter, int axis);

    // 设置二维参数指定轴的坐标值，axis为0时设置U，为1时设置V。
    static void setParameterCoordinate(MyMath::Vector2& parameter, int axis, double value);

    // 将二维参数点序列指定轴整体平移固定值。
    static void shiftPointsCoordinate(std::vector<MyMath::Vector2>& points, int axis, double shift);

    // 将一段Edge采样点在指定周期轴上平移整数个周期，使起点靠近前一参数点。
    static bool alignPeriodicCoordinate(std::vector<MyMath::Vector2>& edgePoints, const MyMath::Vector2& previousPoint,
                                        int axis, double period);

    // 判断两个参数点是否通过具体Surface奇点语义表示同一个拓扑连接位置。
    static bool singularParametersMeet(const Geometry_Surface& surface, const MyMath::Vector2& first,
                                       const MyMath::Vector2& second, const ParametricFaceMeshPolicy& policy, double tolerance);

    // 判断两个参数点是否直接重合或通过合法奇点形成同一个拓扑连接位置。
    static bool parametersConnect(const Geometry_Surface& surface, const MyMath::Vector2& first,
                                  const MyMath::Vector2& second, const ParametricFaceMeshPolicy& policy, double tolerance);

    // 将当前Edge采样点周期对齐到上一Edge终点，并保留合法奇点处的不同UV表示。
    static bool alignEdgeSampleToPrevious(std::vector<MyMath::Vector2>& edgePoints, const MyMath::Vector2& previousPoint,
                                          const Geometry_Surface& surface, const ParametricFaceMeshPolicy& policy, double tolerance);

    // 将闭合Wire全部Edge-use采样并拼接为一个连续参数Ring。
    static bool sampleWire(const Topology_Wire& wire, const Geometry_Surface& surface,
                           const ParametricFaceMeshOptions& options, const ParametricFaceMeshPolicy& policy,
                           std::vector<MyMath::Vector2>& points);

    // 返回Ring在指定参数轴上的平均坐标。
    static double ringCenterCoordinate(const Ring2D& ring, int axis);

    // 返回Ring在指定参数轴上的坐标跨度。
    static double ringCoordinateSpan(const Ring2D& ring, int axis);

    // 将全部Ring在指定周期轴上对齐到同一个连续周期图。
    static bool normalizeRingPeriods(std::vector<Ring2D>& rings, int axis, double period, double tolerance);

    // 计算每个Ring被其他Ring包含的层数，用于even-odd外环和孔洞识别。
    static void calculateRingDepths(std::vector<Ring2D>& rings, double tolerance);

    // 将Ring方向调整为指定的顺时针或逆时针方向。
    static void orientRing(std::vector<MyMath::Vector2>& ring, bool counterClockwise);

    // 返回Ring最右侧顶点索引；X相同时优先较低Y。
    static std::size_t rightmostVertex(const std::vector<MyMath::Vector2>& ring);

    // 比较两个Hole的最右侧顶点X坐标，用于从右向左排序Hole。
    static bool holeRightmostGreater(const std::vector<MyMath::Vector2>& first, const std::vector<MyMath::Vector2>& second);

    // 判断候选桥接线段是否与当前弱简单多边形的非端点边相交。
    static bool bridgeCrossesPolygon(const MyMath::Vector2& first, const MyMath::Vector2& second,
                                     const std::vector<MyMath::Vector2>& polygon, double tolerance);

    // 判断候选桥接线段是否与指定Hole的非端点边相交。
    static bool bridgeCrossesRing(const MyMath::Vector2& first, const MyMath::Vector2& second,
                                  const std::vector<MyMath::Vector2>& ring, double tolerance);

    // 为指定Hole寻找一条位于有效trim区域内且不穿越现有边界的桥接边。
    static bool findBridge(const std::vector<Ring2D>& allRings, const std::vector<MyMath::Vector2>& polygon,
                           const std::vector<std::vector<MyMath::Vector2> >& remainingHoles, std::size_t currentHoleIndex,
                           std::size_t holeVertexIndex, double tolerance, std::size_t& polygonVertexIndex);

    // 将指定Hole通过桥接边缝合到当前弱简单多边形。
    static void stitchHole(std::vector<MyMath::Vector2>& polygon, std::size_t polygonVertexIndex,
                           const std::vector<MyMath::Vector2>& hole, std::size_t holeVertexIndex);

    // 判断指定顶点是否可以作为当前弱简单多边形的合法耳朵。
    static bool isEar(const std::vector<MyMath::Vector2>& polygon, std::size_t index, double tolerance);

    // 删除一个重复或简单共线退化顶点，用于耳切无法继续时消除局部退化。
    static bool removeOneDegenerateVertex(std::vector<MyMath::Vector2>& polygon, double tolerance);

    // 使用耳切算法将弱简单多边形三角化。
    static bool earClip(std::vector<MyMath::Vector2> polygon, double tolerance, std::vector<Triangle2D>& triangles);

    // 将一个外Ring及其直接Hole先桥接为弱简单多边形，再执行耳切三角化。
    static bool triangulateFilledRing(const std::vector<Ring2D>& allRings, const Ring2D& outerRing,
                                      const std::vector<const Ring2D*>& holeRings, double tolerance,
                                      std::vector<Triangle2D>& triangles);

    // 查找或追加指定参数顶点，并返回其索引。
    static unsigned int findOrAppendParameterVertex(const MyMath::Vector2& parameter, double tolerance,
                                                    std::vector<MyMath::Vector2>& vertices);

    // 将参数三角形转换为共享顶点的索引三角形。
    static bool buildIndexedTriangles(const std::vector<Triangle2D>& triangles, double tolerance,
                                      std::vector<MyMath::Vector2>& vertices, std::vector<IndexedTriangle>& indexedTriangles);

    // 判断Surface指定参数处的一阶参数化是否正则。
    static bool parameterIsRegular(const Geometry_Surface& surface, const MyMath::Vector2& parameter);

    // 判断Surface指定参数处是否属于当前Mesher策略认可的参数奇点。
    static bool parameterIsSingular(const Geometry_Surface& surface, const MyMath::Vector2& parameter,
                                    const ParametricFaceMeshPolicy& policy, double tolerance);

    // 返回一条参数网格边的三维中点弦高误差。
    static double surfaceEdgeChordError(const Geometry_Surface& surface,
                                        const std::vector<MyMath::Vector2>& vertices, const EdgeKey& edge);

    // 将一个索引三角形的三条无向边加入边集合。
    static void collectTriangleEdges(const IndexedTriangle& triangle, std::set<EdgeKey>& edges);

    // 返回一个三角形当前被标记为需要切分的边数量。
    static int splitEdgeCount(const IndexedTriangle& triangle, const std::set<EdgeKey>& splitEdges);

    // 当三角形恰有两条边需要切分时补切第三边，保证一致细分规则。
    static void addMissingThirdEdgeForTwoSplitTriangle(const IndexedTriangle& triangle, std::set<EdgeKey>& splitEdges);

    // 返回指定共享边的中点顶点索引，不存在时创建并缓存。
    static unsigned int midpointVertex(const EdgeKey& edge, std::vector<MyMath::Vector2>& vertices,
                                       std::map<EdgeKey, unsigned int>& midpointIndices);

    // 根据当前曲面弦高误差执行一轮无T-junction共享边一致细分。
    static bool refineSurfaceOnce(const Geometry_Surface& surface, double tolerance,
                                  std::vector<MyMath::Vector2>& vertices, std::vector<IndexedTriangle>& triangles, bool& changed);

    // 判断当前参数网格是否仍存在超过曲面弦高误差的共享边。
    static bool surfaceNeedsMoreRefinement(const Geometry_Surface& surface, double tolerance,
                                           const std::vector<MyMath::Vector2>& vertices,
                                           const std::vector<IndexedTriangle>& triangles);

    // 重复执行曲面共享边一致细分直到满足误差或达到最大轮数。
    static bool refineSurface(const Geometry_Surface& surface, const ParametricFaceMeshOptions& options,
                              std::vector<MyMath::Vector2>& vertices, std::vector<IndexedTriangle>& triangles);

    // 判断指定参数三角形映射到三维Surface后是否具有非零面积。
    static bool trianglePositionsAreNonDegenerate(const Geometry_Surface& surface, const MyMath::Vector2& firstParameter,
                                                  const MyMath::Vector2& secondParameter, const MyMath::Vector2& thirdParameter);

    // 判断索引三角形是否引用指定顶点。
    static bool triangleUsesVertex(const IndexedTriangle& triangle, unsigned int vertexIndex);

    // 为奇点顶点从相邻三角形中寻找一个正则参数作为极限法向逼近方向。
    static bool findRegularApproachParameter(unsigned int singularVertexIndex, const Geometry_Surface& surface,
                                             const std::vector<MyMath::Vector2>& parameterVertices,
                                             const std::vector<IndexedTriangle>& triangles,
                                             const ParametricFaceMeshPolicy& policy, double tolerance,
                                             MyMath::Vector2& regularApproachParameter);

    // 判断三角形是否包含当前策略认可的参数奇点。
    static bool triangleContainsSingularParameter(const IndexedTriangle& triangle, const Geometry_Surface& surface,
                                                  const std::vector<MyMath::Vector2>& parameterVertices,
                                                  const ParametricFaceMeshPolicy& policy, double tolerance);

    // 计算指定参数网格顶点符合Face方向的单位法向，并在合法奇点处调用具体奇点处理器。
    static bool evaluateMeshNormal(const Topology_Face& face, unsigned int vertexIndex,
                                   const std::vector<MyMath::Vector2>& parameterVertices,
                                   const std::vector<IndexedTriangle>& triangles,
                                   const ParametricFaceMeshPolicy& policy, double tolerance, MyMath::Vector3& normal);

    // 根据最终参数顶点和索引三角形建立FaceMesh，并处理Face方向和允许的奇点退化三角形。
    static FaceMesh buildFaceMesh(const Topology_Face& face, const std::vector<MyMath::Vector2>& parameterVertices,
                                  const std::vector<IndexedTriangle>& triangles,
                                  const ParametricFaceMeshPolicy& policy, double tolerance);
};

}

#endif // MYBREP_MESH_PARAMETRICFACEMESHERCORE_H