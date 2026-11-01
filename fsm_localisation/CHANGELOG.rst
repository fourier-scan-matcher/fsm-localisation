^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fsm_localisation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-11-01)
------------------
* Declares ``libfftw3-dev`` as a build dependency and ``libfftw3`` as a run
  dependency. ``ros/rosdistro`` #54634 split the ``libfftw3`` rule: that key
  now installs the run time libraries only, and the new key ``libfftw3-dev``
  installs the headers. Thus ``rosdep`` installs each dependency, and the
  install steps and ``docker/Dockerfile`` no longer install ``libfftw3-dev``
  by name or skip a key.
* Switches the translation bound off in ``params_amelioration.yaml``. On the
  rb1 recording, the estimate was up to 0.25 m and 10 degrees out on every
  third frame. There the bound refused 510 to 670 of 2223 frames. It also let
  through 17 to 57 answers more than 0.5 m or 10 degrees out. With the bound
  off, no frame was refused and 1 to 49 answers were that far out. With the
  estimate on the logged pose, the median heading gap fell from 0.587 to 0.503
  degrees.
* Unpacks the spectrum of an odd count of rays correctly. It unpacked each
  spectrum as if its length was even. Thus at an odd length it removed the
  imaginary part of the highest coefficient and gave one coefficient too few.
  This changed each rotation that the matcher found for such a scan. In the
  test room the start was 0.14 m and 4.6 degrees out. From there a scan of 269
  rays now comes to 0.067 degrees and 0.35 cm from the truth. Before, it came
  to 0.23 degrees and 1.1 cm. A scan laid out as the 1073 rays of the rb1
  recording now comes to 0.012 degrees, where it came to 0.39 degrees. At an
  odd length the arithmetic no longer agrees with the ROS 1 package.
* Lays the map out from the first ray of the scan. The matcher cast its window
  of a turn from the start of that window. Thus it matched a scan whose first
  ray was at a different angle up to half a ray out. In the test room, the
  heading of a whole turn symmetric about the heading was 0.5 degrees out. On
  a narrower scan where the turn less its rays is odd, as on the rb1
  recording, the finer casts of the rotation step were half a ray out. Now
  each cast lies along the rays of the scan, at each count of rays. In the
  test room the error is less than 4e-15 radians, and on the rb1 layout less
  than 2e-7.
* ``Corrector::correct`` takes a fifth argument, ``first_ray_angle``. It is
  the angle of the first range from the heading of the sensor, and a caller
  must supply it. The node supplies the ``angle_min`` of the scan.
  ``FSM::Reference`` increases from 32 to 40 bytes.
* Refuses a scan whose ``angle_min`` is not a number.
* Refuses an estimate stamped before zero with a sentence and keeps running.
  The conversion of such a stamp to a time threw an exception that nothing
  caught, and the node aborted at the first such estimate.
* Ignores a scan stamped before zero in odometry mode, with a warning, and
  keeps running. The conversion of such a stamp to a time threw an exception
  that nothing caught, and the node aborted at the first such scan.
* Makes the node's start and stop flag safe to read and write from several
  executor threads at once. The flag was a plain ``bool``, written by
  ``start`` and ``stop`` and read on other executor threads. A ThreadSanitizer
  run of the start, stop and correction tests reported that race seven times
  before and none after.
* Lays the optional map scan over the scan the sensor reported, ray for ray,
  whatever the sensor covers. It spread the scan's ray count over a whole turn
  from half a turn behind the heading, but carried the scan's own angles. Thus
  for a sensor that covers less than a turn, it did not lie over the scan. Each
  ray is now cast along the matching ray of the scan. In the test room it now
  disagrees with a 270 degree scan by 0.026 m a ray, where it disagreed by
  0.56 m.
* No longer says that a non-zero ``rng_seed`` makes a run repeat. The README
  and both parameter files said so. Each executor thread seeds its own
  generator, and any of them may run a match. Thus which draw a recovery gets
  depends on the thread schedule. On a recording from a real robot, three
  odometry runs at ``rng_seed`` 1 over 2000 scans differed from step 79.
* Reads a sensor mounted upside down from the transform tree, with nothing to
  set. The correction read the rotation of the sensor's placement on the robot
  as a heading and nothing more, so a lidar mounted half a turn about its
  forward axis was taken for an upright one and its scan was matched as the
  mirror image of the room. On a recording from a robot whose lidar is mounted
  that way, none of twenty estimates pushed 0.1 m and 5 degrees off the logged
  pose was put back within 0.1 m and 2 degrees of it. The whole rotation is now
  read. Where the sensor's z axis points down the ranges are taken in reverse
  order and laid out from the sensor's heading less the sum of the scan's first
  and last angles, which is exact for any scan the node accepts. Reversing the
  ranges alone would turn the answer by one ray on a scan beginning at minus
  pi. A setup that flipped the scan in its driver while its transform tree
  carries the half turn of roll is now read mirrored, and should drop one of
  the two. The optional map scan now runs in the order the sensor reports its
  rays, where for such a sensor it was drawn as the mirror image of the scan.
* Corrects a defect that answered a narrower sensor confidently and wrongly.
  ``Corrector::correct`` was never told what the sensor covered, so it read the
  map over a whole turn and matched it against a measurement covering less of
  one. That converges rather than failing, so every signal a caller can read
  said the correction was fine. The map is now read over a whole turn at the
  sensor's own angular spacing and windowed to what the sensor could have seen.
  The aperture is a required argument with no default, since a default of a
  whole turn is what let a caller stay silent and be answered wrongly. The node
  derives it from the scan and refuses a scan spanning more than a turn, or one
  not centred on the sensor's heading.
* Records the arithmetic it was built with, where a dependent package reads it.
  Nearly all of the matcher is in a header, so a dependent package compiles that
  header for itself. Two arithmetics over one header put two definitions of one
  inline function into a single process, and which one survives linking is the
  linker's choice. ``FSM_LOCALISATION_BUILT_COMPILE_OPTIONS`` and
  ``FSM_LOCALISATION_BUILT_EXACT_FP`` arrive with the package's own cmake
  configuration, so whoever builds this package settles the arithmetic for
  everything above it.
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
  it always has while all three are off. Available in both modes. All three are
  unconditional in the scan-to-map matcher and absent from the odometry one, and
  the ``fsm`` readme prescribes the gear-up for scan-to-map-scan matching and
  against it for scan matching, so ``config/params_amelioration.yaml`` turns all
  three on and ``config/params_odometry.yaml`` leaves all three off.
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
  scan stamps, so it no longer varies with machine load. A replay of a
  recording gives the same stamps every time. A match that needs the recovery
  search can still give a different pose on each replay.
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
* Adds the ``rng_seed`` parameter, which seeds the recovery search. It does not
  make a run repeat, as the entry on ``rng_seed`` above says.
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
