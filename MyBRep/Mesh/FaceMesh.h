#ifndef MYBREP_MESH_FACEMESH_H
#define MYBREP_MESH_FACEMESH_H

#include <cstddef>
#include <vector>

#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"

namespace MyBRep
{

// Face三角网格中的单个顶点。
// parameter保存原始Surface参数，position和normal保存对应的三维几何数据。
struct FaceMeshVertex
{
    FaceMeshVertex();
    FaceMeshVertex(const MyMath::Vector2& parameterValue, const MyMath::Vector3& positionValue, const MyMath::Vector3& normalValue);

    MyMath::Vector2 parameter; // 顶点对应的Surface参数(u,v)。
    MyMath::Vector3 position;  // 顶点对应的三维曲面点。
    MyMath::Vector3 normal;    // 顶点对应当前Face使用方向的单位法向。
};

// Face离散后的索引三角网格。
// indices每连续三个索引构成一个有向三角形，三角形正面方向与Face法向一致。
class FaceMesh
{
public:
    // 构造空网格。
    FaceMesh();

    /// 状态

    // 判断当前网格是否没有任何顶点和三角形。
    bool isEmpty() const;
    // 判断当前网格是否具有完整且索引有效的三角网格数据。
    bool isValid() const;

    /// 数据

    // 返回网格顶点数量。
    std::size_t vertexCount() const;
    // 返回网格三角形数量。
    std::size_t triangleCount() const;
    // 返回全部顶点。
    const std::vector<FaceMeshVertex>& vertices() const;
    // 返回全部三角形索引。
    const std::vector<unsigned int>& indices() const;

    /// 修改

    // 清空全部顶点和索引。
    void clear();
    // 添加顶点并返回其索引。
    unsigned int addVertex(const FaceMeshVertex& vertex);
    // 添加一个索引三角形。
    void addTriangle(unsigned int first, unsigned int second, unsigned int third);

private:
    std::vector<FaceMeshVertex> m_vertices; // Surface参数、三维位置和法向组成的顶点数组。
    std::vector<unsigned int> m_indices;    // 每三个索引构成一个有向三角形。
};

}

#endif // MYBREP_MESH_FACEMESH_H
