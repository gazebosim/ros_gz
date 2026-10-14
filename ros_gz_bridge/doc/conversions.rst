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
- ``builtin_interfaces.hpp``
- ``geometry_msgs.hpp``
- ``gps_msgs.hpp``
- ``marine_acoustic_msgs.hpp``
- ``nav_msgs.hpp``
- ``rcl_interfaces.hpp``
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
4. **Update the dependencies.**  Add the ROS message package to ``package.xml``
   and to the ``BRIDGE_MESSAGE_TYPES`` list in ``CMakeLists.txt``, and link
   ``${<pkg>_TARGETS}`` into ``${bridge_lib}`` and ``test_utils``.  That list is
   what pulls ``src/convert/<pkg>.cpp`` and the generated factories into the
   build; without the entry, the library fails to link.
5. **Rebuild.**  The code generator in ``bin/`` is wired up through
   ``add_custom_command``, so the per-type factory plumbing is regenerated from
   ``mappings.py`` at *build* time, not on CMake configure.  It emits files for
   every package in ``MAPPINGS``, but only those listed in
   ``BRIDGE_MESSAGE_TYPES`` are compiled.
6. **Add tests.**  Add a self-contained round-trip test in
   ``src/convert/<pkg>_TEST.cpp`` and register it with ``ament_add_gtest``, and
   add ``createTestMsg`` / ``compareTestMsg`` helpers in ``test/utils`` for the
   end-to-end bridge tests.

See :doc:`tutorials/adding_a_message` for the full walkthrough.

Header Index
------------

Full auto-generated pages for the conversion headers:

- :ref:`file_include_ros_gz_bridge_convert.hpp`
- :ref:`file_include_ros_gz_bridge_convert_decl.hpp`

Per-package conversion headers are linked from the same page.
