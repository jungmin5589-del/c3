# v1.14.1 camera runtime publishing fix

Symptom fixed: front/rear camera topic names appear in `ros2 topic list`, but no image is received by rqt_image_view or RViz.

Changes:
- Validate every camera prim before publisher setup.
- Initialize each Isaac Sim Camera, render warm-up frames, then initialize it again.
- Wait for the SDGPipeline to be created before configuring RGB, depth, and CameraInfo publish gates.
- Increase image publisher queue size from 1 to 2.
- Print every camera prim, render product, and ROS topic at startup.
- Add `scripts/check_camera_ros.sh` for message-level diagnostics.

The camera Replicator writers are runtime-generated under the SDGPipeline. Therefore rqt_graph is not the primary camera health check. Use `ros2 topic info -v`, `ros2 topic echo --once`, and `ros2 topic hz`.
