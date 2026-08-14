#ifndef MYBREP_FOUNDATION_DIAGNOSTIC_H
#define MYBREP_FOUNDATION_DIAGNOSTIC_H

#if defined(_MSC_VER)
#define MYBREP_NORETURN __declspec(noreturn)
#define MYBREP_FUNCTION_NAME __FUNCTION__
#else
#define MYBREP_NORETURN [[noreturn]]
#define MYBREP_FUNCTION_NAME __func__
#endif

namespace MyBRep
{
namespace Foundation
{

/// 诊断失败处理

// 输出断言失败信息并终止程序。
MYBREP_NORETURN void reportAssertionFailure(const char* condition, const char* message, const char* file, int line, const char* functionName);

// 输出必要条件失败信息并终止程序。
MYBREP_NORETURN void reportRequirementFailure(const char* condition, const char* message, const char* file, int line, const char* functionName);

}
}

#if defined(NDEBUG)

#define MYBREP_ASSERT(condition) do { (void)sizeof(condition); } while (false)
#define MYBREP_ASSERT_MESSAGE(condition, message) do { (void)sizeof(condition); } while (false)

#else

#define MYBREP_ASSERT(condition) \
    do \
    { \
        if (!(condition)) \
        { \
            MyBRep::Foundation::reportAssertionFailure(#condition, nullptr, __FILE__, __LINE__, MYBREP_FUNCTION_NAME); \
        } \
    } while (false)

#define MYBREP_ASSERT_MESSAGE(condition, message) \
    do \
    { \
        if (!(condition)) \
        { \
            MyBRep::Foundation::reportAssertionFailure(#condition, message, __FILE__, __LINE__, MYBREP_FUNCTION_NAME); \
        } \
    } while (false)

#endif

#define MYBREP_REQUIRE(condition) \
    do \
    { \
        if (!(condition)) \
        { \
            MyBRep::Foundation::reportRequirementFailure(#condition, nullptr, __FILE__, __LINE__, MYBREP_FUNCTION_NAME); \
        } \
    } while (false)

#define MYBREP_REQUIRE_MESSAGE(condition, message) \
    do \
    { \
        if (!(condition)) \
        { \
            MyBRep::Foundation::reportRequirementFailure(#condition, message, __FILE__, __LINE__, MYBREP_FUNCTION_NAME); \
        } \
    } while (false)

#define MYBREP_UNUSED(value) ((void)(value))

#endif // MYBREP_FOUNDATION_DIAGNOSTIC_H