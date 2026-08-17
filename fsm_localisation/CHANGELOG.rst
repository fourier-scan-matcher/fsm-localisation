^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fsm_localisation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* First release of ``fsm_localisation``. One node with two modes, one matcher
  underneath both. ``odometry`` matches each scan against the one before it and
  reports how far the sensor moved. ``amelioration`` matches a scan against a
  ``nav_msgs/msg/OccupancyGrid`` and reports where the sensor actually was,
  correcting a pose estimate somebody else produced.
* The odometry mode is ``fsm_lidar_odometry`` renamed, taken from commit
  ``d343895c73cb535b40a758b3f86a6cfee01574a5``, and is unchanged rather than
  merely equivalent. Every step of the merge that touched the matcher compared
  disassembly before and after, and where anything moved, ran all 97 match steps
  of three recorded scenarios through both builds at full precision. Driven over
  the recordings the predecessor's own proof tree holds, the two agree to
  fourteen decimal places on every published quantity.
* The scan-to-map half is new. Its ray casting is ``range_libc`` by Corey H.
  Walsh, Apache 2.0, vendored under ``include/fsm_localisation/range_libc/`` and
  taken from cbgl's copy of it, cut to three casters. Given the same grid and
  the same rays, this copy and cbgl's answer identically on every one of 705024
  casts. Four defects in cbgl's additions were corrected on the way in and are
  recorded in the readme beside that code.
* Unlike cbgl, the matcher casts into the map at every pose it tries rather than
  matching against one scan frozen at the pose the correction started from. What
  comes back is a corrected pose rather than a displacement.
* Adds the ``mode`` parameter, which has no default: a node that does not know
  which of the two things it does refuses to start and exits rather than guess.
* ``config/params.yaml`` becomes ``config/params_odometry.yaml`` and
  ``config/params_amelioration.yaml``, both keyed by the wildcard ``/**:``
  rather than by a node name, so a file applies whatever the node is called.
  Only the running mode's settings are read, so one file can serve a mode while
  carrying nonsense for the other. Two nodes, one in each mode, run side by side
  from the one launch file.
* The four service names follow the node's name, as they always have, so they
  are now ``fsm_localisation/start`` and so on. Their types and behaviour are
  unchanged. The amelioration mode offers ``start``, ``stop`` and
  ``correct_once`` instead.
* The default topic names follow the node's name too, so what was
  ``fsm_lidar_odometry/pose_estimate`` is now ``fsm_localisation/pose_estimate``
  and the same for ``path_estimate``, ``lo`` and ``initial_pose``.
  ``scan_topic`` still defaults to ``/base_scan``.
* The matcher's entry point is ``FSM::Match::fmt``. It was ``fmtdbh``. This
  matters only to code driving the core directly rather than through the node.
* The core gained a reference handle, so the matcher can be handed either the
  ring of points the odometry mode builds or an occupancy grid behind an
  abstract interface. The ring path through it is unchanged.
* Adds ``enforce_early_gearup``, ``enforce_translation_bound`` and
  ``recover_on_failed_sift``, three behaviours carried across from the
  scan-to-map matcher, all defaulting off so that the matcher behaves exactly as
  it always has while all three are off. Available in both modes.
* Corrects two further defects in the matcher, each characterised by a test that
  failed first: an out of bounds read in the rotation stage, and a read of an
  empty vector reachable through ``recover_on_failed_sift``. The first changes
  the odometry path's reported rotation criterion, which the recorded scenarios
  could not have caught because they compare poses.
* Builds with ``-Ofast -fno-finite-math-only`` rather than bare ``-Ofast``.
  ``-Ofast`` implies ``-ffinite-math-only``, under which a guard against a value
  that is not a number folds away to nothing. Measured cost over 4636 matches:
  26.86 ms a match before, 26.79 ms after, inside run to run noise.
* Contributors: Alexandros Filotheou

The entry below records the release of ``fsm_lidar_odometry``, which this
package's odometry mode is. It is kept as it was written.

Inherited from fsm_lidar_odometry, unreleased
---------------------------------------------
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
