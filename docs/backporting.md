

# A collection of merge requests to be backported #


## Unsorted ##

[Fix "roscpp multithreaded spinners eat up CPU when callbacks take too long"](https://github.com/ros/ros_comm/pull/2377)

https://github.com/ros/ros_comm/pull/2370

https://github.com/ros/ros_comm/pull/2369

https://github.com/ros/ros_comm/pull/2365 - XmlRpcServer is being retired so maybe it is not needed.

https://github.com/ros/ros_comm/pull/2363

https://github.com/ros/ros_comm/pull/2357

https://github.com/ros/ros_comm/pull/2355

https://github.com/ros/ros_comm/pull/2351 - rosbag fix


[Added init_options::NoSimTime to forcefully disable subscribing to /clock](https://github.com/ros/ros_comm/pull/2342)

[roscpp: Fixed remapping of private parameters in anonymous nodes](https://github.com/ros/ros_comm/pull/2325) - 

[Add missing repeat_latched initialization ](https://github.com/ros/ros_comm/pull/2314)

[Support TCP/IP and UDS(Unix Domain Socket) hybrid transport for roscp…](https://github.com/ros/ros_comm/pull/2309) - new unix domain socket transport.

[Fix for timer/srv-server finalization issue 2283](https://github.com/ros/ros_comm/pull/2284) - ignored.

[Fix EINTR handling in XmlRpcDispatch::work](https://github.com/ros/ros_comm/pull/2278)


[Log errors in XmlRpcpp](https://github.com/ros/ros_comm/pull/2277)


Stopped parsing list at: https://github.com/ros/ros_comm/pulls?page=12&q=is%3Apr+is%3Aclosed


## Merged to noetic/one ##

BAG, Merged [Ensure latched messages are updated on every split](https://github.com/ros/ros_comm/pull/2261)

BAG, Merged [Exposed record snapshot feature for command line](https://github.com/ros/ros_comm/pull/2254)

[Fix printing XmlRpcValue with GTest](https://github.com/ros/ros_comm/pull/2224)

[use recursive mutex to fix dead lock](https://github.com/ros/ros_comm/pull/2209) - fixes to rospararm requester. Simular thing is needed to MasterLink. Though valgrind/helgrind does not like recursive mutexes, but deadlock is still deadly. Not relevant anymore, since requests are done through completely other means.

[keep the persistent connection only if rosmaster supports http1.1](https://github.com/ros/ros_comm/pull/2208) - nice update to XmlRpcClient. Though it will be update to use new http_toos package.

[fix char signedness issue for test_base64 on ARM and RISC-V](https://github.com/ros/ros_comm/pull/2205) - very relevant.

[some fixes for the --repeat_latched option](https://github.com/ros/ros_comm/pull/2201) - very relevant

[Noetic: Fix XMLRPC endless loop](https://github.com/ros/ros_comm/pull/2185) - very relevant for XML parsing.

[[xmlrpcpp] Fix build when gtest is not available](https://github.com/ros/ros_comm/pull/2177) - probably relevant.

[Avoid uncatchable exception in bagthread [noetic] ](https://github.com/ros/ros_comm/pull/2151/files) - relevant.

[fix bug #2123: Do not raise exception if socket is busy in TCPROSTransport ](https://github.com/ros/ros_comm/pull/2131) - relevant

[Fix for deadlock issue 1980](https://github.com/ros/ros_comm/pull/2121) - relevant

[Portable fix to recent Windows build breaks.](https://github.com/ros/ros_comm/pull/2110) - not very relevant, since XmlRpc is seriously reworked.

[Add missing library depends](https://github.com/ros/ros_comm/pull/2108) - not relevant anymore.

[fix getNumPublishers() to only count fully connected](https://github.com/ros/ros_comm/pull/2107) - very relevant

[Replace message assertion with logging in order to have release modes](https://github.com/ros/ros_comm/pull/2096) - relevant

[set call_finished_ with true for each call inside callFinished](https://github.com/ros/ros_comm/pull/2074) - relevant

[cached parameter should be unsubscribed](https://github.com/ros/ros_comm/pull/2068) - relevant

[Trap for overly large input to XmlRPCPP ](https://github.com/ros/ros_comm/pull/2065) - need to be reworked and adapted to new xmlrpc, probably for security reasons. Contains #2064


[Gracefully stop recording upon SIGTERM and SIGINT](https://github.com/ros/ros_comm/pull/2038) - relevant for rosbag recorder.

[Fix Lost Wake Bug in ROSOutAppender](https://github.com/ros/ros_comm/pull/2033) - relevant, simple.

[use fully qualified ceil() in condition_variable.h](https://github.com/ros/ros_comm/pull/2025) - not  relevant. Custom condition variables were dropped later in https://github.com/ros/ros_comm/pull/1878

[Remove 'using namespace' from condition_variable.h](https://github.com/ros/ros_comm/pull/2020) - not relevant. Custom conditino variables were dropped later in https://github.com/ros/ros_comm/pull/1878

[Fix subscription busy wait melodic](https://github.com/ros/ros_comm/pull/2014) - not relevant. Current implementation already matches latest ros-one:obesse.

[Use an internal implementation of boost::condition_variable with monotonic clock](https://github.com/ros/ros_comm/pull/2011) - not relevant, reverted in https://github.com/ros/ros_comm/pull/1878

[XmlRpcValue::\_doubleFormat should be used during write](https://github.com/ros/ros_comm/pull/2003) - relevant

[Allow mixing latched and unlatched publishers.](https://github.com/ros/ros_comm/pull/1991) - relevant for latched publishers

[Changed is_async_connected to use epoll when available](https://github.com/ros/ros_comm/pull/1983) - very relevant

[Added const versions of XmlRpcValue converting operators.](https://github.com/ros/ros_comm/pull/1978) - fixed in alternative way.

[[noetic] Fixing Windows build break.](https://github.com/ros/ros_comm/pull/1961/files) - probably fixed already

[check if async socket connect is success or failure before read and write in TransportT](https://github.com/ros/ros_comm/pull/1954) - relevant

[Fix a bug that using a destroyed connection object.](https://github.com/ros/ros_comm/pull/1950) - relevant.

[Add latch param to throttle](https://github.com/ros/ros_comm/pull/1944) - do we have topic throttle?

[Fix bug that connection drop signal related funtion throw a bad_weak …](https://github.com/ros/ros_comm/pull/1940) - boost::signals were replaced but some issues can be te same

[Use an internal implementation of boost::condition_variable with monotonic clock](https://github.com/ros/ros_comm/pull/1932) - probably was already implemented in another MR


[Check if enough FDs are free, instead counting the total free FDs.](https://github.com/ros/ros_comm/pull/1929) - probably not relevant anymore

[Remove extra \n in ROS_DEBUG.](https://github.com/ros/ros_comm/pull/1925/files) - minor fix

[Add rosout integration test](https://github.com/ros/ros_comm/pull/1924) - python-based test for rosout. Not sure I need it.

[Fix #923 (use undefined dynamic_lookup on OSX)](https://github.com/ros/ros_comm/pull/1923) cmake fix for roslz4 on OS X

[Fix brief description comments after members](https://github.com/ros/ros_comm/pull/1920) - minor fix for comment blocks in rosbag

[add default assignment operator for various classes](https://github.com/ros/ros_comm/pull/1888) - relevant

[catch polymorphic exceptions by reference](https://github.com/ros/ros_comm/pull/1887) - relevant

[Drop custom implementation of boost::condition_variable to fix busy-wait spinning](https://github.com/ros/ros_comm/pull/1878) - this is probably beyond base point

[[Windows][melodic-devel] Conditionally guard sys/socket.h for Windows](https://github.com/ros/ros_comm/pull/1876) - minor fix

[[rosbag] Catch exceptions by const ref.](https://github.com/ros/ros_comm/pull/1874) - probably need it.

[Do not display error message if poll yields EINTR](https://github.com/ros/ros_comm/pull/1868) - fixed in different way. Poll implementation will return Error::Ok in case of EINTR.


## Dropped ##

Dropped [Added feature to enable or disable logs through service call](https://github.com/ros/ros_comm/pull/2256)


roscpp, Dropped (ix #2249: Topic statistics do not handle well ROS time jumping )[https://github.com/ros/ros_comm/pull/2250/files]


roscpp, dropped [Statistics deadlock fix](https://github.com/ros/ros_comm/pull/2246/files)

[rosbag::Recorder: allow pausing of message recording](https://github.com/ros/ros_comm/pull/2228)

[Use ros time instead of boost local_time to add date time string to bag filename](https://github.com/ros/ros_comm/pull/2211) - not sure miniros needs this

[Improve performance of ShapeShifter::read() ](https://github.com/ros/ros_comm/pull/2200) - strange case. Though reducing number of allocations is always helpful.


[rosbag: introduce external index file to speed up reading](https://github.com/ros/ros_comm/pull/2180) - nice idea to speed up playing bags, but PR had some bugs

[fix error in base64EncodedSize()](https://github.com/ros/ros_comm/pull/2160) - probably it is needed for UDP transport. 

[github action for noetic ](https://github.com/ros/ros_comm/pull/2145) - reference for running github CI

[Fix for #152 add a an optional timeout on service calls](https://github.com/ros/ros_comm/pull/2144) - very needed

[Add a rosbag tool 'transform' to split, merge and transform bag files](https://github.com/ros/ros_comm/pull/2134) - nice to have

[Fix Building Encrypted Bags on Mac](https://github.com/ros/ros_comm/pull/2114) - probably relevant for native MAC

[Fix throttle publish first message](https://github.com/ros/ros_comm/pull/2088) - may be relevant

[Add a new mutex for dropped_ ](https://github.com/ros/ros_comm/pull/2061) - it is probably applied in another MR. But is relevant.

[[Windows][noetic] Fixing Time jumped forward & 100% CPU occupied issues](https://github.com/ros/ros_comm/pull/1962) - need to study

[Unsubscribe cached parameter when XmlRpcManager shuts down.](https://github.com/ros/ros_comm/pull/1945)


[Option to record with custom freq](https://github.com/ros/ros_comm/pull/1935) - nice to have, but not important.

[Avoid warning messages when a publisher unregisters](https://github.com/ros/ros_comm/pull/1933) - huge change. Hard to verify.

[Added const indexer for xmlrpc (#1759) ](https://github.com/ros/ros_comm/pull/1931) - not interested
