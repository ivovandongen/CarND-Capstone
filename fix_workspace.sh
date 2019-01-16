echo "Fixing Udacity Self Driving car workspace..."

echo "Installing missing dependencies..."
apt-get -y install ros-kinetic-dbw-mkz
pip install catkin_pkg
pip install catkin
cd ros/
catkin_make

echo "Done...should be good to go now! "
