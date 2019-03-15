Usage:
-----

Start the client:
```
roscd refills_test_perception_interface
rosrun refills_test_perception_interface test_detection_action_interface.py -p ./data
```

Start the test server:

```
rosrun refills_test_perception_interface test_server
```

The client will start reading the file pairs from the directory igven as a param and call the action server;
