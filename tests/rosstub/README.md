Minimal stand-ins for rclpy, message packages and tf2_ros, just enough to execute the ROS 2 node
logic (here: rik_ros base_driver_node) in-process without a ROS installation. Publishing is
synchronous and every timer runs on one simulated clock. The real ROS 2 launch test runs in CI
(.github/workflows/ci.yml, ros:jazzy container).
