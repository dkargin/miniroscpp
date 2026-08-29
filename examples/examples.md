# Examples #

Examples are added to build by setting cmake option MINIROS_BUILD_EXAMPLES=ON.
These examples follow regular ROS tutorials.

 - bag_read - reading data directly from bag.
 - bag_write - writing BAG from C++ code.
 - listener - miniros-based listener, similar to listener tutorial from regular ROS1.
 - talker - miniros-based talker, similar to talker tutorial from regular ROS1
 - bench_talker - benchmark, based on regular talker. It measures timings for initializing ROS and sending messages.
 - log_demo
 - param_form_demo - embeddable HTTP parameter form (settings Apply/Cancel)

## Websocket example

1. Start server in terminal: `./build/bin/websocket_server` The server will start on port 8080 (or another port if 8080 is unavailable).
2. Open the HTML test page, http://localhost:8080 or URL reported in terminal.
3. Test the connection:
   - Click the "Connect" button in the HTML page
   - Enter a message in the text field
   - Click "Send" to send the message
   - The server will echo back the message with "Echo: " prefix
   - Check the server console output to see received messages

## Param form example

Demonstrates a custom HTTP page handler that owns most of the HTML and embeds a
`ParameterForm` over a `ParameterCollection`. Dirty fields show `*`. Apply can be
rejected with a generic message and per-field errors (demo: 4x4 + 90 FPS).

1. Build with `-DMINIROS_BUILD_EXAMPLES=ON`.
2. Run: `./build/bin/param_form_demo` (or your build tree's binary path).
3. Open `http://localhost:8080/` (or the port printed in the terminal).
4. Change fields (see `*`), Apply to commit, or try 4x4 + 90 FPS for a rejection.
