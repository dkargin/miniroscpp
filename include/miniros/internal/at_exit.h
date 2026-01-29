#ifndef MINIROS_UTILS_AT_EXIT
#define MINIROS_UTILS_AT_EXIT

#include <functional>

namespace miniros {

/// Helper class to call some method when gone out of scope.
class AtExit
{
public:
    AtExit(std::function<void(void)> exitFn) : m_exitFn(exitFn) {}
    ~AtExit()
    {
        if (m_exitFn)
            m_exitFn();
    }
private:
    std::function<void(void)> m_exitFn;
};

}


#endif