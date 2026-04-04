# Examples #

Examples are added to build by setting cmake option MINIROS_BUILD_EXAMPLES=ON.
These examples follow regular ROS tutorials.

 - bag_read - reading data directly from bag.
 - bag_write - writing BAG from C++ code.
 - listener - miniros-based listener, similar to listener tutorial from regular ROS1.
 - talker - miniros-based talker, similar to talker tutorial from regular ROS1
 - bench_talker - benchmark, based on regular talker. It measures timings for initializing ROS and sending messages.
 - log_demo 

## Websocket example

1. Start server in terminal: `./build/bin/websocket_server` The server will start on port 8080 (or another port if 8080 is unavailable).
2. Open the HTML test page, http://localhost:8080 or URL reported in terminal.
3. Test the connection:
   - Click the "Connect" button in the HTML page
   - Enter a message in the text field
   - Click "Send" to send the message
   - The server will echo back the message with "Echo: " prefix
   - Check the server console output to see received messages
