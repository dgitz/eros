# VS Code Setup
1. Install the Recommended VS Code extensions by opening the Extensions Sidebar and typing `@recommended`.


# Generate Doxygen HTML

```
git checkout master
cd /home/robot/catkin_ws/src/eros
doxygen Doxyfile
git commit --allow-empty -m "Trigger rebuild"
git push
```