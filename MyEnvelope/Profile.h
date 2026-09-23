#ifndef MYMATH_PROFILE_H
#define MYMATH_PROFILE_H

#include "MyMath/Vector2.h"

#include <cstddef>
#include <vector>

namespace MyMath
{

constexpr double DefaultEpsilon = 1.0e-12;

class Profile
{
public:
    Profile() = default;
    virtual ~Profile() = default;

    // 返回弧长 l 处的单位法向量，l 应在 [0, length()] 内。
    virtual Vector2 normalAtL(double l, double epsilon = DefaultEpsilon) const
    {
        (void)l;
        (void)epsilon;
        return Vector2(0.0, 0.0);
    }

    // 返回母线总弧长。
    virtual double length() const { return 0.0; }

    // 返回弧长 l 处的 r 坐标。
    virtual double r(double l) const { (void)l; return 0.0; }

    // 返回弧长 l 处的 h 坐标。
    virtual double h(double l) const { (void)l; return 0.0; }

    // 返回弧长 l 处的点 (r, h)。
    Vector2 pointAtL(double l) const { return Vector2(r(l), h(l)); }
};

class ProfileVector : public Profile
{
public:
    ProfileVector() = default;

    explicit ProfileVector(const std::vector<Profile>& profiles)
        : m_profileVec(profiles)
    {
        init();
    }

    Vector2 normalAtL(double l, double epsilon = DefaultEpsilon) const override
    {
        if (l < 0.0 || l > m_length)
        {
            return Vector2(0.0, 0.0);
        }

        const int index = getProfile(l);
        if (index < 0)
        {
            return Vector2(0.0, 0.0);
        }

        return m_profileVec[static_cast<std::size_t>(index)].normalAtL(l, epsilon);
    }

    double length() const override { return m_length; }

    double r(double l) const override
    {
        if (l < 0.0 || l > m_length)
        {
            return -1.0;
        }

        const int index = getProfile(l);
        if (index < 0)
        {
            return -1.0;
        }

        return m_profileVec[static_cast<std::size_t>(index)].r(l);
    }

    double h(double l) const override
    {
        if (l < 0.0 || l > m_length)
        {
            return -1.0;
        }

        const int index = getProfile(l);
        if (index < 0)
        {
            return -1.0;
        }

        return m_profileVec[static_cast<std::size_t>(index)].h(l);
    }

    // 按弧长 l 查找所在子轮廓的下标。
    // 成功时把 l 就地转换为段内局部弧长，返回子轮廓下标；
    // 越界或列表为空时返回 -1，l 保持不变。
    int getProfile(double& l) const
    {
        if (l < 0.0 || l > m_length)
        {
            return -1;
        }

        if (m_profileVec.empty())
        {
            return -1;
        }

        const int lastIndex = static_cast<int>(m_profileVec.size()) - 1;

        for (int i = 0; i <= lastIndex; ++i)
        {
            const double segLen = m_profileVec[static_cast<std::size_t>(i)].length();

            // 落到本段内，或已是最后一段（吸收浮点误差），都归为这一段。
            if (l < segLen || i == lastIndex)
            {
                return i;
            }

            l -= segLen;
        }

        return -1;
    }

private:
    void init()
    {
        m_length = 0.0;

        for (const auto& profile : m_profileVec)
        {
            m_length += profile.length();
        }
    }

private:
    std::vector<Profile> m_profileVec;
    double m_length = 0.0;
};

} // namespace MyMath

#endif // MYMATH_PROFILE_H