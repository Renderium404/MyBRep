#ifndef MYENVELOPE_ENVELOPE_REVOLVEDENVELOPEBRANCH_H
#define MYENVELOPE_ENVELOPE_REVOLVEDENVELOPEBRANCH_H

namespace MyEnvelope
{

// 标识回转面包络方程A*cos(theta)+B*sin(theta)+C=0的两条解析分支。
enum class RevolvedEnvelopeBranch
{
    Plus = 0, // theta = phi + acos(-C / D)。
    Minus     // theta = phi - acos(-C / D)。
};

}

#endif // MYENVELOPE_ENVELOPE_REVOLVEDENVELOPEBRANCH_H
