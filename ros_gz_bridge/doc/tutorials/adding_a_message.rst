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
4. ``package.xml`` and ``CMakeLists.txt`` — declare the new dependency, add it
   to the ``BRIDGE_MESSAGE_TYPES`` list, and link it.
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
- **Nested messages.**  Reuse the existing conversion functions rather than
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

Then add the package to the ``BRIDGE_MESSAGE_TYPES`` list in ``CMakeLists.txt``.
This list — not ``mappings.py`` — is what actually switches the new package on
in the build.  The ``foreach`` loop over it calls ``find_package``, adds
``src/convert/example_msgs.cpp`` to the library sources, adds the generated
``generated/factories/example_msgs.{cpp,hpp}`` to the build, generates the
per-package test subscriber, and passes the package to
``ament_export_dependencies``.  Registering the mapping in Step 1 alone is not
enough.

.. code-block:: cmake
   :caption: CMakeLists.txt

   set(BRIDGE_MESSAGE_TYPES
     builtin_interfaces
     actuator_msgs
     example_msgs
     geometry_msgs
     # ... existing packages ...
   )

Because that loop already runs ``find_package(${package_name} QUIET REQUIRED)``,
you do **not** need a separate ``find_package(example_msgs REQUIRED)`` call.
The link libraries are still listed explicitly, so add the message targets
there:

.. code-block:: cmake
   :caption: CMakeLists.txt

   target_link_libraries(${bridge_lib}
     PUBLIC
       # ... existing deps ...
       ${example_msgs_TARGETS}
   )

Add ``${example_msgs_TARGETS}`` to the ``test_utils`` target in the
``BUILD_TESTING`` block as well, so the shared test helpers can build against
the new type.

.. note::

   If you skip the ``BRIDGE_MESSAGE_TYPES`` entry, the generator still writes
   ``generated/factories/example_msgs.cpp`` — it walks ``mappings.py``, not the
   CMake list — so seeing that file appear is *not* proof the build picked it
   up.  The file is never compiled, while the generated ``get_factory.cpp``
   still includes ``factories/example_msgs.hpp`` and calls
   ``get_factory__example_msgs()``.  The library then fails at link time with an
   undefined reference to that symbol.

Re-run the build (``colcon build --packages-select ros_gz_bridge``) and confirm
both that ``bin/ros_gz_bridge_generate_factories`` produced
``build/ros_gz_bridge/generated/factories/example_msgs.cpp`` and that
``libros_gz_bridge.so`` links.

Step 5 — Tests
--------------

``ros_gz_bridge`` has two layers of tests that both need work for the new type,
and they do **not** share fixtures.  The end-to-end bridge tests (5.3) run over
real transport and are driven by the ``createTestMsg`` / ``compareTestMsg``
helpers in ``test/utils``.  The in-process conversion tests (5.2) are
self-contained and do not use those helpers at all.

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

Do the same for the ROS side in ``test/utils/ros_test_msg.{hpp,cpp}``.  Keep the
two ``createTestMsg`` helpers producing equivalent content: the end-to-end tests
in 5.3 publish what one side creates and check it with the other side's
``compareTestMsg``, so any mismatch surfaces there as a bridge failure.

Both files are compiled into the ``test_utils`` static library, which every
generated publisher/subscriber program links.  Nothing in 5.2 uses them.

5.2 Conversion round-trip test
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The in-process convert tests include only their own convert header and build
their fixtures inline — see ``src/convert/ros_gz_interfaces_TEST.cpp``, which
declares a local ``::testing::Test`` subclass and its own ``constexpr`` expected
values.  ``rcl_interfaces_TEST.cpp`` and ``sensor_msgs_TEST.cpp`` follow the
same shape.  Populate a ROS message, convert to Gazebo, convert back, and assert
field by field:

.. code-block:: cpp
   :caption: src/convert/example_msgs_TEST.cpp

   #include <gtest/gtest.h>

   #include <ros_gz_bridge/convert/example_msgs.hpp>

   constexpr auto kExpectedValue = 42;
   constexpr const char * kExpectedLabel = "hello";

   TEST(ExampleMsgTest, RoundTripRosGzRos)
   {
     example_msgs::msg::ExampleMessage ros_in;
     ros_in.value = kExpectedValue;
     ros_in.label = kExpectedLabel;

     gz::msgs::ExampleMessage gz_mid;
     ros_gz_bridge::convert_ros_to_gz(ros_in, gz_mid);
     EXPECT_EQ(kExpectedValue, gz_mid.value());
     EXPECT_EQ(kExpectedLabel, gz_mid.label());

     example_msgs::msg::ExampleMessage ros_out;
     ros_gz_bridge::convert_gz_to_ros(gz_mid, ros_out);
     EXPECT_EQ(kExpectedValue, ros_out.value);
     EXPECT_EQ(kExpectedLabel, ros_out.label);
   }

Then register the test in ``CMakeLists.txt`` inside the ``BUILD_TESTING`` block,
following ``test_sensor_msgs``.  Linking ``${bridge_lib}`` is what supplies the
conversion symbols — ``src/convert/example_msgs.cpp`` is already compiled into
the library by the Step 4 list edit — and the only include directory the test
needs is the package's own ``include``:

.. code-block:: cmake

   ament_add_gtest(test_example_msgs
     ${PROJECT_SOURCE_DIR}/src/convert/example_msgs_TEST.cpp
   )
   target_link_libraries(test_example_msgs
     ${bridge_lib}
     ${example_msgs_TARGETS}
     gtest
     gtest_main
   )
   target_include_directories(test_example_msgs
     PRIVATE
     ${PROJECT_SOURCE_DIR}/include
   )

5.3 End-to-end bridge test
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``BUILD_TESTING`` block generates its test programs into the build tree —
``build/ros_gz_bridge/generated/test/``, not ``test/`` — and builds them as the
executables ``test_gz_publisher``, ``test_ros_publisher``,
``test_gz_subscriber``, and ``test_ros_subscriber``, installed to
``lib/ros_gz_bridge``.

``gz_publisher.cpp``, ``ros_publisher.cpp``, and ``gz_subscriber.cpp`` are each
generated as a *single* file covering the whole mapping table, so those three do
pick up the new type from Step 1 alone — provided the Step 5.1 helpers exist and
``${example_msgs_TARGETS}`` was added to ``test_utils`` in Step 4.  Without that
link edit, ``test_ros_publisher`` fails to compile on the new message header.

``test_ros_subscriber`` behaves differently, and this is the case worth
watching.  It is built from ``ros_subscriber_files``: the hand-written
``test/subscribers/ros_subscriber.cpp`` plus one generated
``<pkg>_subscriber.cpp`` *per entry in* ``BRIDGE_MESSAGE_TYPES``.  The generator
writes ``example_msgs_subscriber.cpp`` either way — it walks ``mappings.py``,
not the CMake list — but the file is only compiled into the target if the
package is in the list.  Skip the Step 4 edit and ``test_ros_subscriber`` still
builds and still passes, just with no ``ROSSubscriberTest`` case for the new
type at all.  A green test run is not evidence the new type is covered; check
that the case name appears in the output.

.. warning::

   Do not hand-write ``test/subscribers/example_msgs_subscriber.cpp``.  The
   CMake loop already adds a generated file of that basename to
   ``test_ros_subscriber``, so adding yours to ``ros_subscriber_files`` gives
   the target two translation units defining the same
   ``TEST(ROSSubscriberTest, ...)`` cases — a redefinition error — and leaving
   it out means it is never compiled.

To customise subscribe-side behaviour, extend the ``MyTestClass`` /
``TestSubOnly`` templates in ``test/subscribers/ros_subscriber.hpp``, or the
``test/resource/ros_pkg_subscriber.cpp.em`` template that emits the per-package
cases.  Touch ``test/launch/test_ros_subscriber.launch.py`` /
``test_gz_subscriber.launch.py`` only if the launch wiring needs extending.

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
- [ ] ``package.xml`` updated with the new ROS dependency.
- [ ] ``CMakeLists.txt``: package added to ``BRIDGE_MESSAGE_TYPES`` and
  ``${<pkg>_TARGETS}`` linked into ``${bridge_lib}`` and ``test_utils``.
- [ ] ``createTestMsg`` / ``compareTestMsg`` helpers added for both the ROS
  and Gazebo sides.
- [ ] Round-trip gtest added and registered in ``CMakeLists.txt``.
- [ ] End-to-end publisher/subscriber tests pass **and** a
  ``ROSSubscriberTest`` case for the new type actually appears in the
  ``test_ros_subscriber`` output.
- [ ] ``README.md`` mapping table updated.
- [ ] ``colcon test`` is green.

Further Reading
---------------

- :doc:`../conversions` — reference listing of all conversion headers.
- :doc:`../user_api` — the C++ types you build on top of.
- ``bin/ros_gz_bridge_generate_factories`` — the code generator driven by
  ``mappings.py``.
