Adding a New Message to the Bridge
===================================

This tutorial walks through adding support for a new ROS ↔ Gazebo message pair
in ``ros_gz_bridge``.  As a worked example, we will bridge
``example_msgs/msg/ExampleMessage`` to ``gz.msgs.ExampleMessage``.  Replace the
type and field names with your own as you follow along.

You will end up editing or adding files in six places:

1. ``ros_gz_bridge/mappings.py`` — the ROS ↔ GZ type mapping table used by the
   code generator at build time.
2. ``include/ros_gz_bridge/convert/<pkg>.hpp`` — template specializations.
3. ``src/convert/<pkg>.cpp`` — the conversion implementations.
4. ``package.xml`` and ``CMakeLists.txt`` — declare the new dependency and link
   it.
5. ``test/utils/gz_test_msg.{hpp,cpp}`` and
   ``test/utils/ros_test_msg.{hpp,cpp}`` — ``createTestMsg`` /
   ``compareTestMsg`` helpers for the new types.
6. ``README.md`` — keep the mapping table in sync (optional but encouraged).

Prerequisites
-------------

- The ROS message (``example_msgs/msg/ExampleMessage``) already exists.
- The Gazebo protobuf message (``gz.msgs.ExampleMessage``) is provided by a
  package reachable via ``gz_msgs_vendor`` or a custom vendor package.
- You can build and run the ``ros_gz_bridge`` tests locally:

  .. code-block:: bash

     colcon build --packages-select ros_gz_bridge
     colcon test --packages-select ros_gz_bridge --event-handlers console_direct+

Step 1 — Register the mapping
-----------------------------

Open ``ros_gz_bridge/mappings.py`` and add the new pair under the ROS package
key.  Keep the entries alphabetically sorted to minimise diff noise.

.. code-block:: python
   :caption: ros_gz_bridge/mappings.py

   MAPPINGS = {
       # ...
       'example_msgs': [
           Mapping('ExampleMessage', 'ExampleMessage'),
       ],
       # ...
   }

The left-hand string is the ROS message name (``example_msgs/msg/ExampleMessage``).
The right-hand string is the Gazebo message class name (not the fully-qualified
``gz.msgs.ExampleMessage``; just the class).  The code generator uses this
table to emit the per-type factory plumbing on the next CMake configure.

A ROS type may appear in multiple ``Mapping`` entries if several Gazebo types
map to it — see ``nav_msgs`` → ``Odometry`` / ``OdometryWithCovariance`` for an
example.

Step 2 — Declare the specialization
------------------------------------

If ``example_msgs`` does not yet have a convert header, create one:

.. code-block:: cpp
   :caption: include/ros_gz_bridge/convert/example_msgs.hpp

   #ifndef ROS_GZ_BRIDGE__CONVERT__EXAMPLE_MSGS_HPP_
   #define ROS_GZ_BRIDGE__CONVERT__EXAMPLE_MSGS_HPP_

   #include <ros_gz_bridge/convert_decl.hpp>

   #include <example_msgs/msg/example_message.hpp>
   #include <gz/msgs/example_message.pb.h>

   namespace ros_gz_bridge
   {

   template<>
   void
   convert_ros_to_gz(
     const example_msgs::msg::ExampleMessage & ros_msg,
     gz::msgs::ExampleMessage & gz_msg);

   template<>
   void
   convert_gz_to_ros(
     const gz::msgs::ExampleMessage & gz_msg,
     example_msgs::msg::ExampleMessage & ros_msg);

   }  // namespace ros_gz_bridge

   #endif  // ROS_GZ_BRIDGE__CONVERT__EXAMPLE_MSGS_HPP_

Also add the new header to the umbrella include:

.. code-block:: cpp
   :caption: include/ros_gz_bridge/convert.hpp

   #include <ros_gz_bridge/convert/example_msgs.hpp>

Step 3 — Implement both conversions
------------------------------------

Every mapping must be **bidirectional**: if the message can only flow one way
the factory scaffolding still expects both specializations to link.  Implement
both in ``src/convert/example_msgs.cpp``:

.. code-block:: cpp
   :caption: src/convert/example_msgs.cpp

   #include <ros_gz_bridge/convert/example_msgs.hpp>

   namespace ros_gz_bridge
   {

   template<>
   void
   convert_ros_to_gz(
     const example_msgs::msg::ExampleMessage & ros_msg,
     gz::msgs::ExampleMessage & gz_msg)
   {
     gz_msg.set_value(ros_msg.value);
     gz_msg.set_label(ros_msg.label);
   }

   template<>
   void
   convert_gz_to_ros(
     const gz::msgs::ExampleMessage & gz_msg,
     example_msgs::msg::ExampleMessage & ros_msg)
   {
     ros_msg.value = gz_msg.value();
     ros_msg.label = gz_msg.label();
   }

   }  // namespace ros_gz_bridge

Implementation guidelines
~~~~~~~~~~~~~~~~~~~~~~~~~

- **Copy every field that has a clean mapping.**  If a field has no
  counterpart on the other side, leave it at its default value and document
  the drop in a comment.
- **Units and frames.**  ROS is typically SI with REP-103 frame conventions;
  Gazebo is typically SI with protobuf defaults.  Convert angles, handedness,
  and axis orientation as needed.
- **Nested messages.**  Re-use the existing conversion functions rather than
  inlining field-by-field copies.  For example, ``Header`` conversion lives in
  ``convert/std_msgs.cpp`` and is called by every stamped message.
- **Repeated fields.**  Resize the ROS vector and loop over the protobuf
  ``*_size()`` / ``*(i)`` accessors (or the reverse for ``ros_to_gz``).

Step 4 — Dependencies and CMake
--------------------------------

Add ``example_msgs`` as a runtime dependency in ``package.xml``:

.. code-block:: xml
   :caption: package.xml

   <depend>example_msgs</depend>

The CMake build uses the mapping table to generate factory code at configure
time, so there is no explicit ``add_library`` change needed for the new source
file as long as it lives under ``src/convert/``.  However, you do need to
declare the ROS message package:

.. code-block:: cmake
   :caption: CMakeLists.txt

   find_package(example_msgs REQUIRED)

   ament_target_dependencies(${PROJECT_NAME}
     # ... existing deps ...
     example_msgs
   )

Re-run CMake (``colcon build --packages-select ros_gz_bridge``) and confirm
that ``bin/ros_gz_bridge_generate_factories`` produced
``build/ros_gz_bridge/generated/factories/example_msgs.cpp``.

Step 5 — Tests
--------------

``ros_gz_bridge`` has two layers of tests that both need entries for the new
type.  The first exercises the pure conversion round-trip; the second
exercises the end-to-end bridge over real transport.

5.1 Add ``createTestMsg`` / ``compareTestMsg`` helpers
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Populate a deterministic message and write a comparator that asserts every
field matches:

.. code-block:: cpp
   :caption: test/utils/gz_test_msg.hpp

   void createTestMsg(gz::msgs::ExampleMessage & _msg);
   void compareTestMsg(const std::shared_ptr<gz::msgs::ExampleMessage> & _msg);

.. code-block:: cpp
   :caption: test/utils/gz_test_msg.cpp

   void createTestMsg(gz::msgs::ExampleMessage & _msg)
   {
     _msg.set_value(42);
     _msg.set_label("hello");
   }

   void compareTestMsg(const std::shared_ptr<gz::msgs::ExampleMessage> & _msg)
   {
     gz::msgs::ExampleMessage expected;
     createTestMsg(expected);
     EXPECT_EQ(expected.value(), _msg->value());
     EXPECT_EQ(expected.label(), _msg->label());
   }

Do the same for the ROS side in ``test/utils/ros_test_msg.{hpp,cpp}``.  Keep
the two ``createTestMsg`` helpers producing equivalent content — that is the
whole point of the round-trip test.

5.2 Conversion round-trip test
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For in-process conversions, follow the pattern of
``src/convert/ros_gz_interfaces_TEST.cpp``: construct a ROS message with
``createTestMsg``, convert to Gazebo, then convert back, and call
``compareTestMsg`` on the result.

.. code-block:: cpp
   :caption: src/convert/example_msgs_TEST.cpp

   #include <gtest/gtest.h>

   #include <ros_gz_bridge/convert/example_msgs.hpp>

   #include "utils/gz_test_msg.hpp"
   #include "utils/ros_test_msg.hpp"

   TEST(ExampleMsgTest, RoundTripRosGzRos)
   {
     example_msgs::msg::ExampleMessage ros_in;
     ros_gz_bridge::testing::createTestMsg(ros_in);

     gz::msgs::ExampleMessage gz_mid;
     ros_gz_bridge::convert_ros_to_gz(ros_in, gz_mid);

     auto ros_out = std::make_shared<example_msgs::msg::ExampleMessage>();
     ros_gz_bridge::convert_gz_to_ros(gz_mid, *ros_out);

     ros_gz_bridge::testing::compareTestMsg(ros_out);
   }

Then register the test in ``CMakeLists.txt`` inside the ``BUILD_TESTING``
block:

.. code-block:: cmake

   ament_add_gtest(test_example_msgs
     ${PROJECT_SOURCE_DIR}/src/convert/example_msgs_TEST.cpp
     ${PROJECT_SOURCE_DIR}/test/utils/gz_test_msg.cpp
     ${PROJECT_SOURCE_DIR}/test/utils/ros_test_msg.cpp
   )
   target_link_libraries(test_example_msgs
     ${PROJECT_NAME}
     ${GTEST_LIBRARIES}
   )
   ament_target_dependencies(test_example_msgs
     example_msgs
   )

5.3 End-to-end bridge test
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The build generates ``test/gz_publisher``, ``test/gz_subscriber``,
``test/ros_publisher``, and ``test/ros_subscriber`` from ``mappings.py``.
Because you already updated the mapping table in Step 1, those generated
programs pick up the new type automatically.

You only need to touch ``test/subscribers/ros_subscriber.cpp`` (or create a new
``<pkg>_subscriber.cpp``) if the new type requires custom subscribe-side
handling, and ``test/launch/test_ros_subscriber.launch.py`` /
``test_gz_subscriber.launch.py`` if the test launch wiring needs extending.

Run the full suite:

.. code-block:: bash

   colcon test --packages-select ros_gz_bridge --event-handlers console_direct+
   colcon test-result --verbose

Step 6 — README
---------------

``README.md`` contains a human-readable mapping table.  Add a row for the new
pair so users can discover it without reading ``mappings.py``.  The ordering
mirrors ``MAPPINGS`` — keep it consistent.

Verification Checklist
----------------------

Before opening a pull request:

- [ ] ``mappings.py`` entry added and alphabetized.
- [ ] Header specialization declared in ``convert/<pkg>.hpp`` and included
  from ``convert.hpp``.
- [ ] Both ``convert_ros_to_gz`` and ``convert_gz_to_ros`` implementations are
  provided.
- [ ] ``package.xml`` and ``CMakeLists.txt`` updated with the new ROS
  dependency.
- [ ] ``createTestMsg`` / ``compareTestMsg`` helpers added for both the ROS
  and Gazebo sides.
- [ ] Round-trip gtest added and registered in ``CMakeLists.txt``.
- [ ] End-to-end publisher/subscriber tests pass.
- [ ] ``README.md`` mapping table updated.
- [ ] ``colcon test`` is green.

Further Reading
---------------

- :doc:`../conversions` — reference listing of all conversion headers.
- :doc:`../user_api` — the C++ types you build on top of.
- ``bin/ros_gz_bridge_generate_factories`` — the code generator driven by
  ``mappings.py``.
