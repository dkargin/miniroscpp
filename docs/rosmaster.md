# ROS Master protocol #

## getPublishedTopics ##

Provides a list of published topics.


req[0] = string this_node::getName()
req[1] = ""


rep: array of:
    string topic_name, string topic_type

## getSystemState ##

Provides a list of active ROS nodes

req[0] = ownNodeName

rep[] - array of structs 
TODO: Continue this.


## setParam ##

Sets value on rosparam server

Request = {
    string ownNodeName;
    string key;
    RpcValue value;
}

## loopupNode ##

# Node XMLRPC protocol #

## getSubscriptions ##

