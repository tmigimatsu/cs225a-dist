PUMA Redis Driver
=================

This driver provides a Redis interface for communicating with the QNX machine. Instead of directly controlling torques, you will be specifying task-specific parameters like position and orientation.

Installation
------------

1. Uncomment the following line cs225a.git/CMakeLists.txt file to build the driver.

   ```add_subdirectory(src/kuka_iiwa)```

2. Make

   ```./make.sh```

Usage
-----

1. Make sure that you are using the same redis keys in your controller as in the driver. The keys can be imported with:

   ```#include "puma_driver/RedisClient.h"```

   By default, the keys you should read and write in the controller are:
	- "cs225a::puma::sensors::q"          Read the joint positions
	- "cs225a::puma::sensors::dq"         Read the joint velocities
	- "cs225a::puma::tasks::control_mode" Write the control mode
	- "cs225a::puma::tasks::command_data" Write the command data
	- "cs225a::puma::tasks::kp"           Write the position gains
	- "cs225a::puma::tasks::kv"           Write the velocity gains

2. When writing commands to Redis, your keys MUST BE SET ATOMICALLY. Otherwise, you may get race conditions and the Puma will go out of control. For example:

	```
	redis.mset({
		{Puma::KEY_CONTROL_MODE, "GOTO"},
		{Puma::KEY_COMMAND_DATA, RedisClient::encodeEigenMatrix(x_des)},
		{Puma::KEY_KP, RedisClient::encodeEigenMatrix(Kp)},
		{Puma::KEY_KV, RedisClient::encodeEigenMatrix(Kv)}
	});
	```

3. Read puma_driver/RedisClient.h and puma_driver/puma_demo_main.cpp for details on how to interface with the Puma.

4. Run the driver WHILE KEEPING YOUR HAND ON THE E-STOP.

   ```
   cd bin
   ./make.sh
   ./puma_driver
   ```

5. Run your controller

6. When you are done, stop the driver (Ctrl+C) and your controller
