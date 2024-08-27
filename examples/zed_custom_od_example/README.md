# ZED Custom Object Detection Example

This example demonstrates how to use the Custom Object Detection feature of the ZED SDK with the ZED ROS 2 Wrapper.
In this example a new component is created which subscribes to ZED Left image topics, performs AI inference with a custom AI engine, then publishes back the bounding boxes to the ZED Node.

To minimize latency, the example shows how to leverace Composition and Intra Process Communication, two advanced features available in ROS 2.

