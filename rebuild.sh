rm -rf install/ log/ build/
colcon build --symlink-install
source install/setup.bash
