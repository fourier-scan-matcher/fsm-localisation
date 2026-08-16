^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fsm_lo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* First release. Ports the package from ROS 1 Kinetic to ROS 2, and is the
  first version published to a ROS distribution. The ROS 1 version was only
  ever available from source, so there is no earlier changelog to continue.
* The package is renamed from ``fsm_lo`` to ``fsm_lidar_odometry``. The node,
  the launch file, the default node name, the include directory, the C++
  namespace and the four service names all follow it. Topic and parameter
  names are unchanged.
* The four services take and return ``std_srvs/srv/Trigger`` instead of taking
  and returning nothing, so callers now get a success flag and a message. The
  service names are unchanged.
* Output is stamped from the incoming scan rather than from the clock at the
  moment of processing. The reported twist follows from the interval between
  scan stamps, so it no longer varies with machine load, and replaying a
  recording gives the same output every time.
* The scan subscription's reliability and depth are exposed as the
  ``scan_qos_reliability`` and ``scan_qos_depth`` parameters. The defaults
  mirror ROS 1, so a ``best_effort`` sensor driver needs
  ``scan_qos_reliability`` set to match or no scans arrive.
* ``size_scan`` accepts ``0``, now the default, which matches every ray the
  scan carries instead of requiring a fixed ray count. A scan carrying fewer
  ranges than a non-zero ``size_scan`` is refused with a warning instead of
  being read past its end.
* Adds the ``ray_search`` parameter, selecting between the new ``angular``
  search, now the default, and the ``windowed`` search the algorithm shipped
  with. Over 13908 matches at ``size_scan: 360``, ``angular`` completes a match
  in 17.9 ms against ``windowed``'s 25.4 ms, median.
* Adds the ``rng_seed`` parameter. A non-zero value pins the recovery search so
  a run can be reproduced.
* Parameters outside their permitted range are refused at start-up with an
  explanation instead of tripping an assertion that release builds compile
  away.
* The matching core reports through an installable diagnostic sink rather than
  writing to a terminal. The node forwards it to the ROS log. Stage timings are
  compiled out unless the core is built with ``FSM_LO_TRACE``.
* The node runs on a multi-threaded executor, so a service call is not held up
  behind a scan being matched.
* Frame id defaults no longer carry a leading slash, which tf2 rejects.
* ``std_msgs/Header`` has no sequence number in ROS 2, so published messages no
  longer carry one.
* Corrects eight defects present in the ROS 1 version:

  * The rotation matrix accumulating the trajectory mixed single and double
    precision, so it was not a rotation. Its determinant was 1.7e-8 from unity,
    and since it is composed once per scan the error grew along the whole path.
  * The initial pose supplied through ``set_initial_pose`` was built in single
    precision, degrading it to about seven significant figures.
  * The convergence criterion of the translation stage took a single precision
    square root of a double.
  * Gap filling read one element before the start of an empty list whenever a
    scan contained no invalid returns at all, which crashed the node.
  * With ``pose_estimate_topic`` unset, the fallback was written to the wrong
    field, so the node threw during construction and could not start at all
    without that parameter.
  * Frame id fallbacks carried a leading slash, which tf2 rejects.
  * Two assertions checked that an unsigned value was at least zero.
  * Output was stamped from the wall clock rather than from the scan.

* Contributors: Alexandros Filotheou
