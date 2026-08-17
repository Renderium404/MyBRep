#include <iostream>
#include <string>
#include <vector>

#include "MyMath/Vector2.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"
#include "MyBRep/Tool/Query/FaceClassifier2D.h"

namespace
{

const double TestTolerance = 1.0e-8; // Face UV分类专项测试统一使用1e-8参数空间几何容差。

class TestContext
{
public:
    TestContext() : m_passed(0), m_failed(0) {}

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

    int passed() const { return m_passed; }
    int failed() const { return m_failed; }

private:
    int m_passed;
    int m_failed;
};

void testSingleRectangle(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);

    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(0.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Inside,
                   "Rectangle center inside");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(6.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Outside,
                   "Rectangle outside");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(5.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Boundary,
                   "Rectangle Edge boundary");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(5.0, 4.0), TestTolerance) == MyBRep::FaceUVClassification::Boundary,
                   "Rectangle Vertex boundary");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(5.0 + TestTolerance * 0.5, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Boundary,
                   "Rectangle boundary tolerance band");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(5.0 + TestTolerance * 4.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Outside,
                   "Rectangle outside beyond tolerance");
}

// 使用同向Wire验证Hole语义完全来自even-odd嵌套关系而不是Wire方向。
void testTwoNestedWires(TestContext& context)
{
    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(MyBRep::Modeling::createRectangle(10.0, 8.0));
    wires.push_back(MyBRep::Modeling::createRectangle(4.0, 2.0));
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(wires, TestTolerance);

    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(4.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Inside,
                   "Two-Wire region between boundaries inside");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(0.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Outside,
                   "Two-Wire nested center outside by even-odd");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(2.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Boundary,
                   "Inner Wire boundary");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(6.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Outside,
                   "Two-Wire exterior outside");
}

// 三层嵌套验证每跨过一条trimming boundary都翻转Inside/Outside。
void testThreeNestedWires(TestContext& context)
{
    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(MyBRep::Modeling::createRectangle(12.0, 10.0));
    wires.push_back(MyBRep::Modeling::createRectangle(8.0, 6.0));
    wires.push_back(MyBRep::Modeling::createRectangle(2.0, 2.0));
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(wires, TestTolerance);

    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(5.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Inside,
                   "Three-Wire outer band inside");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(3.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Outside,
                   "Three-Wire middle band outside");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(0.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Inside,
                   "Three-Wire inner island inside");
}

// Face方向只影响面法向和边界使用方向，不应改变UV区域集合。
void testReversedFace(TestContext& context)
{
    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(MyBRep::Modeling::createRectangle(10.0, 8.0));
    wires.push_back(MyBRep::Modeling::createRectangle(4.0, 2.0));
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(wires, TestTolerance).reversed();

    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(4.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Inside,
                   "Reversed Face inside invariant");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(0.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Outside,
                   "Reversed Face nested outside invariant");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(-5.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Boundary,
                   "Reversed Face boundary invariant");
}

// 单Edge完整圆P-Curve验证曲线边界的Inside/Boundary/Outside。
void testCircleFace(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createCircle(5.0), TestTolerance);

    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(0.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Inside,
                   "Circle Face center inside");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(5.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Boundary,
                   "Circle Face boundary");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(6.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Outside,
                   "Circle Face outside");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(0.0, 4.9), TestTolerance) == MyBRep::FaceUVClassification::Inside,
                   "Circle Face near-top inside");
}

// 没有显式trimming Wire时，完整Surface自然参数域就是Face区域。
void testUntrimmedFace(TestContext& context)
{
    const MyBRep::Topology_Face trimmed = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);
    const MyBRep::Topology_Face face(trimmed.geometryResource());

    context.expect(face.wireCount() == 0, "Untrimmed Face setup");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(0.0, 0.0), TestTolerance) == MyBRep::FaceUVClassification::Inside,
                   "Untrimmed Plane parameter inside");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(1000000.0, -1000000.0), TestTolerance) == MyBRep::FaceUVClassification::Inside,
                   "Untrimmed Plane distant parameter inside");
}

// 射线与矩形水平边同高但查询点不在该边界段上时仍应正确分类，不受共线退化影响。
void testHorizontalRayDegeneracy(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);

    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(-6.0, -4.0), TestTolerance) == MyBRep::FaceUVClassification::Outside,
                   "Horizontal ray collinear with boundary remains outside");
    context.expect(MyBRep::classifyFaceUV(face, MyMath::Vector2(0.0, 3.0), TestTolerance) == MyBRep::FaceUVClassification::Inside,
                   "Horizontal ray ordinary inside");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Face UV Classifier 2D Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testSingleRectangle(context);
    testTwoNestedWires(context);
    testThreeNestedWires(context);
    testReversedFace(context);
    testCircleFace(context);
    testUntrimmedFace(context);
    testHorizontalRayDegeneracy(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}