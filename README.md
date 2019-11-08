# ROS2 Grasp Library Tutorials

To build and release tutorials

```bash
git clone https://github.com/intel/ros2_grasp_library.git
cd ros2_grasp_library
git checkout master
rm -rf docs
cd grasp_tutorials
sphinx-build . ../docs
cd ../grasp_utils/robot_interface
doxygen Doxyfile
cp -a build/* ../docs/api
git add ../docs/
git commit -s -m "gh-pages: update docs"

cd ../
git checkout gh-pages
git cherry-pick master
# check the gh-pages with a local browser
# commit the changes and push to remote gh-pages
# clean up master branch
# git checkout master
# git reset --hard HEAD^

```
