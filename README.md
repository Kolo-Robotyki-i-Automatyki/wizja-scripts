To run:

```console
source install/setup.bash
colcon build --symlink-install --packages-select camera_kra
ros2 run camera_kra camera_subscriber_node
```

Make sure the packages are present, or set them up before building:

```console
python3 -m venv --system-site-packages ../ros2_venv_experimental
source ../ros2_venv_experimental/bin/activate
pip3 install -r requirements.txt
```

If it fails, check is venv is used:
```console
head -1 install/camera_kra/lib/camera_kra/camera_subscriber_node
```

If it doesn't, you can use a hack to fix it (you only need to do it once):

```console
sed -i '1s|.*|#!/usr/bin/env python3|' install/camera_kra/lib/camera_kra/camera_subscriber_node
```
