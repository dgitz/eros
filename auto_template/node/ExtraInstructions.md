Now perform the following:
1. Move the generated content include into your package include path (Follow ROS conventions and move to include/<PackageName>/).
2. Move the generated content nodes into your package nodes path (Follow ROS conventions and move to nodes/<PackageName>).
3. Update the nodes CMakeLists file with adding your new Node folder.
4. In the generated Node Class header, update the following entries: 
    * MAJOR_RELEASE_VERSION
    * MINOR_RELEASE_VERSION
    * BUILD_NUMBER
    * FIRMWARE_DESCRIPTION
5. In the generated Node Class cpp file, update the following entries:
    * diagnostic_types.push_back(...) // See eros_Definitions.h for appropriate values.
6. In the generated Node Launch file, change the package name of the node to your package.
