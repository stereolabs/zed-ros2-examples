
### Launch the multi-camera example

After building and sourcing, launch the dual ZED-X Mini setup (defaults to namespace `camera` and camera names `static_zedxmini_side` and `static_zedxmini_center`):

```bash
source ~/ros2_ws/install/setup.bash
sudo systemctl restart zed_x_daemon
ZED_Explorer -a
export DISPLAY=:0 # For remote ssh -X
xhost +local:
ros2 launch zed_multi_camera zed_multi_camera.launch.py \
  cam_models:='[zedxm,zedxm]' \
  cam_serials:='[58338256,51933055]'
```

This publishes topics under:
- `/camera/static_zedxmini_side/...`
- `/camera/static_zedxmini_center/...`
