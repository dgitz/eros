# Generate Doxygen HTML

```
git checkout master
cd /home/robot/catkin_ws/src/eros
doxygen Doxyfile
git commit --allow-empty -m "Trigger rebuild"
git push
```