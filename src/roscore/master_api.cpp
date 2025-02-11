//
// Created by dkargin on 2/9/25.
//

#include <algorithm>

#include "master_api.h"

namespace miniros {
#ifdef USE_DEPRECATED
constexpr static std::string SEP = "/";
constexpr static std::string PRIV_NAME = "~";

/*std::string get_ros_namespace(std::string env = null)
{
    if(env == null)
        env = "";
    return make_global_ns(env.get(ROS_NAMESPACE, GLOBALNS));
}

static std::string make_caller_id()
{
    return make_global_ns(ns_join(get_ros_namespace(), name));
}*/

std::string Names::make_global_ns(std::string name)
{
    if (is_private(name))
        name = SEP + name;
    if (!is_global(name))
        name = SEP + name;
    if (!name.EndsWith(SEP))
        name = name + SEP;
    return name;
}

bool Names::is_global(const std::string& name)
{
    return name.size() >0 && name.starts_with(SEP);
}

bool Names::is_private(const std::string& name)
{
    return name.size() >0 && name.starts_with(PRIV_NAME);
}

static std::string ns(const std::string& name)
{
    if (name=="")
    {
        return SEP;
    }
    if (name.EndsWith(SEP))
        return SEP;
    return name.Split('/')[0] + "/";
}

static std::string ns_join(std::string ns, std::string name)
{
    if(is_private(name) || is_global(name))
        return name;
    if(ns == PRIV_NAME)
        return PRIV_NAME + name;
    if(ns == "")
        return name;
    if(ns.EndsWith("/"))
        return ns + name;
    return ns + SEP + name;
}

static std::string canonicalize_name(std::string name)
{
    if (name == "" || name == SEP)
        return name;

    std::string[] str = name.Split('/');
    std::string rtn = "";
    for (int i = 0; i < str.size(); i++)
    {
        if(str[i].size() > 1)
            rtn += "/" + str[i];
    }
    return rtn;
}

std::string Names::resolve_name(std::string name, std::string _namespace, std::map<std::string,std::string> remappings = null)
{
    if (name.empty())
        return ns(_namespace);
    std::string resolved_name;
    name = canonicalize_name(name);
    if(name.StartsWith("/"))
        resolved_name = name;
    else if(is_private(name))
        resolved_name = canonicalize_name(_namespace + SEP + name.Replace("~",""));
    else
        resolved_name = ns(_namespace + name);

    if (remappings != null && remappings.ContainsKey(resolved_name))
        return remappings[resolved_name];
    else
        return resolved_name;
}

#endif

} // namespace miniros
