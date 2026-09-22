#include <algorithm>
#include <cmath>
#include <vector>

#include <QApplication>
#include <QDebug>
#include <QTimer>
#include "MyMath/CoordinateSystem.h"
#include "MyMath/MathUtils.h"
#include "MyMath/Quaternion.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Instance/Solid.h"
#include "MyBRep/Modeling/Edge/EdgeModeling.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Shell/ShellModeling.h"
#include "MyBRep/Modeling/Solid/SolidModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Geometry/Surface/Geometry_BSplineSurface.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

class Proflie
{
public:
    Proflie(){}
    ~Proflie(){}

private:
    //我们用三维存储轮廓基于xOYy平面的二维轮廓
    //旋转轴为y轴，
    //注意，实际生成时的旋转轴为Z轴
    void init()
    {

    }

private:
    MyBRep::Topology_Wire profile;
};



int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    MyBRep::Display::BRepViewerWidget window;
    window.resize(1000, 700);
    return app.exec();
}



