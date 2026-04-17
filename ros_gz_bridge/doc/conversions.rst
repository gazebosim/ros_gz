Message Conversions
===================

Every bridged message type has a dedicated pair of C++ conversion functions
specialized on the ROS type and the Gazebo type:

.. code-block:: cpp

   namespace ros_gz_bridge {

   template<typename ROS_T, typename GZ_T>
   void convert_ros_to_gz(const ROS_T & ros_msg, GZ_T & gz_msg);

   template<typename ROS_T, typename GZ_T>
   void convert_gz_to_ros(const GZ_T & gz_msg, ROS_T & ros_msg);

   }  // namespace ros_gz_bridge

Declarations are in ``include/ros_gz_bridge/convert_decl.hpp`` and per-type
specializations are split across headers under ``include/ros_gz_bridge/convert/``:

- ``actuator_msgs.hpp``
- ``geometry_msgs.hpp``
- ``gps_msgs.hpp``
- ``marine_acoustic_msgs.hpp``
- ``nav_msgs.hpp``
- ``ros_gz_interfaces.hpp``
- ``rosgraph_msgs.hpp``
- ``sensor_msgs.hpp``
- ``std_msgs.hpp``
- ``tf2_msgs.hpp``
- ``trajectory_msgs.hpp``
- ``vision_msgs.hpp``

The convenience header ``include/ros_gz_bridge/convert.hpp`` includes all of
them.

Adding a New Conversion
-----------------------

1. **Update the mapping table.**  Add the new ROS/GZ pair to
   :data:`ros_gz_bridge.mappings.MAPPINGS` in ``ros_gz_bridge/mappings.py``.
2. **Declare the specialization.**  Add a declaration in the appropriate
   ``convert/<pkg>.hpp`` header (or create a new one):

   .. code-block:: cpp

      template<>
      void convert_ros_to_gz(
        const my_msgs::msg::Foo & ros_msg,
        gz::msgs::Foo & gz_msg);

      template<>
      void convert_gz_to_ros(
        const gz::msgs::Foo & gz_msg,
        my_msgs::msg::Foo & ros_msg);

3. **Implement both directions.**  Add the definitions under
   ``src/convert/<pkg>.cpp``.  The functions should copy every field that has
   a sensible mapping and leave untranslatable fields at their default values.
4. **Regenerate the factories.**  The code generator in ``bin/`` rebuilds the
   per-type factory plumbing from ``mappings.py`` on the next CMake
   configure.
5. **Update the dependencies.**  Add any new ROS message package to
   ``package.xml`` and ``CMakeLists.txt``.
6. **Add tests.**  Extend the gtest suite in ``test/`` with a round-trip test
   for the new type.

Header Index
------------

Full auto-generated pages for the conversion headers:

- :ref:`file_include_ros_gz_bridge_convert.hpp`
- :ref:`file_include_ros_gz_bridge_convert_decl.hpp`

Per-package conversion headers are linked from the same page.
