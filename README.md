<h1 align='center' style="text-align:center; font-weight:bold; font-size:2.0em;letter-spacing:2.0px;"> FSM: Correspondenceless scan-matching of panoramic 2D range scans </h1>

<div align="center">

[![ieeexplore.ieee.org](https://img.shields.io/badge/IEEE/RSJ_IROS_2022_paper-00629B)](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9981228)
[![youtube.com](https://img.shields.io/badge/1'_presentation-YouTube-FF0000)](https://www.youtube.com/watch?v=hB4qsHCEXGI)
[![github.com](https://img.shields.io/badge/pdf_presentation-333333)](https://mozilla.github.io/pdf.js/web/viewer.html?file=https://raw.githubusercontent.com/phd-li9i/fsm_presentation_iros22/master/main.pdf)

</div>

<h3 align="center">
    <a href="#the-two-modes">I/O</a>
    <span> · </span>
    <a href="#installation">install</a>
    <span> · </span>
    <a href="#run">run</a>
</h3>

`fsm_localisation` is a ROS 2 package written in C++ that matches 2D lidar scans. One node, two modes, one matcher underneath both. Odometry mode expects a sensor whose field of view is 360 degrees. Amelioration mode also reads a sensor that covers less.

**Odometry mode** matches each scan against the one before it and reports how far the sensor moved. No map and no pose estimate are needed.

**Amelioration mode** matches a scan against an occupancy grid and reports where the sensor actually was, correcting a pose estimate somebody else produced.

**A sensor that does not see all the way round is read in amelioration mode.** The map is read over a whole turn at the sensor's own angular spacing and only the part of it the sensor could have seen is compared. The scan has to say what it covers, through `angle_increment` and a ray count, and has to be centred on the sensor's heading; a scan that spans more than a turn, or that begins somewhere else, is refused at the point of use rather than answered wrongly. Odometry mode does not read what a scan covers. It lays every scan out over a whole turn, so it expects a sensor that sees all the way round. It does not refuse a narrower one.

<p align="center">
  <img src="https://i.imgur.com/hUsBImy.png">
</p>

Both are achieved by scan-matching _without establishing correspondences_ between elements of the input scans or their properties, but by leveraging the range signal's periodicity. Hence FSM may exploit properties of the Discrete Fourier Transform. These two pillars support the robustness of FSM's pose error against (a) sensor noise and (b) distance between consecutive poses, as exhibited in the figure which summarises key experiments below.

## Why use FSM

![Experimental results at a glance](https://i.imgur.com/GvFlHgF.png)

Table of Contents
=================

* [Requirements](#requirements)
* [Installation](#installation)
  * [With Docker](#with-docker)
  * [From source](#from-source)
* [Run](#run)
  * [Launch](#launch)
  * [Both modes at once](#both-modes-at-once)
  * [Call](#call)
* [The two modes](#the-two-modes)
  * [Odometry mode](#odometry-mode)
  * [Amelioration mode](#amelioration-mode)
* [Parameters](#parameters)
  * [Shared by both modes](#shared-by-both-modes)
  * [Odometry mode only](#odometry-mode-only)
  * [Amelioration mode only](#amelioration-mode-only)
* [What the map costs and what it bounds](#what-the-map-costs-and-what-it-bounds)
* [The borrowed ray casting](#the-borrowed-ray-casting)
* [Diagnostics](#diagnostics)
* [Coming from fsm_lidar_odometry](#coming-from-fsm_lidar_odometry)
* [Upgrading from the ROS 1 version](#upgrading-from-the-ros-1-version)
* [Motivation and Under the hood](#motivation-and-under-the-hood)

## Requirements

ROS 2 Lyrical and a compiler with C++23 support. Beyond ROS the package needs FFTW3, CGAL and Eigen3. All three are declared in the package manifest, so `rosdep` pulls them in without being asked.

This package is MIT, but two libraries it links are not. The CGAL components it uses are GPL v3 and FFTW is GPL v2 or later. A binary built from these sources therefore carries GPL v3 terms if you redistribute it.

The vendored ray casting under `include/fsm_localisation/range_libc/` is Apache 2.0 and is governed by the licence file beside it rather than by this package's MIT terms. See [The borrowed ray casting](#the-borrowed-ray-casting).

## Installation

Two ways. There is no apt package and no published image: this package is built from these sources.

### With Docker

The image carries ROS 2 Lyrical, every dependency and a built workspace, so the host needs nothing but Docker. It is a runtime image: it holds what the node needs to run and no visualisation or simulation tools, which keeps it a little over two gigabytes.

```bash
docker compose -f docker/docker-compose.yml build
```

Two build arguments change the arithmetic, and both default to off, which is what this package ships as:

```bash
docker compose -f docker/docker-compose.yml build \
  --build-arg EXACT_FP=ON --build-arg DETERMINISTIC_FFT=ON
```

`EXACT_FP=ON` forbids the compiler from reassociating floating point arithmetic, so two compilers give comparable numbers. `DETERMINISTIC_FFT=ON` makes FFTW pick its plan by heuristic rather than by timing itself, so two runs of one binary give the same numbers. Both cost speed. Turn them on to compare builds, not to run a robot.

**Whoever builds this package settles the arithmetic for everything above it.** The choice is recorded where a dependent package reads it, so a package built against this one follows it rather than making its own. See [As a dependency of your own package](#as-a-dependency-of-your-own-package).

`docker/shared-input` and `docker/shared-output` on the host are mounted into the container, for feeding it a recording and taking one back.

### From source

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this repository>
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fsm_localisation
```

### As a dependency of your own package

The matcher is usable without the node. Name this package in your manifest and link the target you want: `fsm_localisation::fsm_localisation_core` for the matcher on its own, which pulls in no ROS, or `fsm_localisation::fsm_localisation_interface` for the node's own class.

```cmake
find_package(fsm_localisation REQUIRED)

target_link_libraries(your_target fsm_localisation::fsm_localisation_core)
```

Each target carries this package's headers and its own dependencies, so Eigen, CGAL and FFTW3 do not have to be found again on your side.

One thing to know before trusting the numbers. Nearly all of the matcher is in the header, so your own compiler flags govern the arithmetic of what you call, not the flags this package was built with. That is a hazard rather than a freedom: two sets of flags over one header put two definitions of one inline function into a single process, and which one survives linking is the linker's choice rather than anybody's intention.

So this package records what it was built with, and a dependent package should follow it rather than choose again:

```cmake
find_package(fsm_localisation REQUIRED)
add_compile_options(${FSM_LOCALISATION_BUILT_COMPILE_OPTIONS})
```

`FSM_LOCALISATION_BUILT_EXACT_FP` is set beside it, true or false, for anything that wants to report the choice rather than follow it. The transform planning choice needs neither: it travels as a compile definition on the exported library targets and reaches your translation units through the link.

The figures quoted here were measured at the settings this package's own `CMakeLists.txt` sets, and the tolerance you can hold it to at other settings has not been characterised.

## Run

### Launch

The odometry mode is what the launch file does with no arguments:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch fsm_localisation fsm_localisation.launch.xml
```

The amelioration mode needs its own parameter file and a name of its own:

```bash
ros2 launch fsm_localisation fsm_localisation.launch.xml \
  node_name:=fsm_localisation_amelioration \
  params:=$(ros2 pkg prefix --share fsm_localisation)/config/params_amelioration.yaml
```

With Docker, in either mode. The container holds a built workspace but its main
process is a shell, so it is started and then launched into:

```bash
docker compose -f docker/docker-compose.yml up -d
docker compose -f docker/docker-compose.yml exec --user fsm_localisation \
  fsm_localisation bash -lc 'ros2 launch fsm_localisation fsm_localisation.launch.xml'
```

Both arguments matter. `--user fsm_localisation` is needed because the container's main process drops to that user but an `exec` does not, and only that user's shell has the workspace on its path. `bash -lc` is needed because the environment comes from that shell's profile rather than from the image.

**`mode` has no default.** A node that does not know which of the two things it does has nothing sensible to fall back on, so it refuses to start and exits rather than guess. Anything launching this node must set it, which in practice means passing one of the two parameter files.

The launch file takes three arguments: `namespace`, which defaults to `/`; `node_name`, which defaults to `fsm_localisation`; and `params`, which defaults to `config/params_odometry.yaml`.

### Both modes at once

Two nodes, one in each mode, from the same launch file twice. This is what the two parameter files are for and it is verified by doing it.

```bash
ros2 launch fsm_localisation fsm_localisation.launch.xml
ros2 launch fsm_localisation fsm_localisation.launch.xml \
  node_name:=fsm_localisation_amelioration \
  params:=$(ros2 pkg prefix --share fsm_localisation)/config/params_amelioration.yaml
```

Two nodes may not share a name, which is what `node_name` is for. The two parameter files also carry different topic names for the same setting, because both modes publish a pose on `pose_estimate_topic` and the two are different message types: run both from one file's topics and incompatible messages land on one topic.

Both parameter files are keyed by the wildcard `/**:` rather than by a node name, so a file applies whatever the node is called and renaming the node cannot silently drop every setting in it.

Only the running mode's settings are read. That is what lets one file serve a mode while carrying nonsense for the other.

### Call

Launching puts the node into stand-by; it processes nothing until told to. To start:

```bash
ros2 service call /fsm_localisation/start std_srvs/srv/Trigger
```

## The two modes

The executable is `fsm_localisation_interface_node` in both. It runs on a multi-threaded executor, so a service call cannot be held up behind a scan being matched.

Every service is node relative, so it follows the node's name. The names below assume the default name `fsm_localisation`.

### Odometry mode

`fsm_lidar_odometry` under a new name, and provably unchanged: driven over the recordings that package's own proof tree holds, the two agree to fourteen decimal places on every published quantity, with every orientation and every timestamp identical.

#### Subscribed topics

| Topic | Type | Utility |
| --- | --- | --- |
| `scan_topic` | `sensor_msgs/msg/LaserScan` | 2d scans are published here, each taken as a whole turn |
| `initial_pose_topic` | `geometry_msgs/msg/PoseWithCovarianceStamped` | optional, for setting the very first pose estimate to something other than the origin |

`initial_pose_topic` is not a standing subscription. One message is read from it when `set_initial_pose` is called, and nothing listens to it otherwise.

#### Published topics

| Topic | Type | Utility |
| --- | --- | --- |
| `pose_estimate_topic` | `geometry_msgs/msg/PoseStamped` | the pose accumulated from every match so far, relative to `global_frame_id` |
| `path_estimate_topic` | `nav_msgs/msg/Path` | the whole estimated trajectory, relative to `global_frame_id` |
| `lo_topic` | `nav_msgs/msg/Odometry` | the displacement between the last two scans |

Every message is stamped with the timestamp of the scan that produced it, not with the clock at the moment it was produced. A replay of a recording therefore gives the same stamps every time. A match that needs the recovery search can still give a different pose on each replay, see `rng_seed` under Parameters.

#### Services offered

All take `std_srvs/srv/Trigger` and return a success flag and a message.

| Service | Utility |
| --- | --- |
| `fsm_localisation/start` | commences node functionality |
| `fsm_localisation/stop` | halts node functionality, node remains alive |
| `fsm_localisation/clear_estimated_trajectory` | clears the vector of estimated poses and returns the accumulated pose to the origin |
| `fsm_localisation/set_initial_pose` | node waits for one message on `initial_pose_topic`, sets the initial pose from it, and returns. Waits for as long as it takes |

#### Transforms published

```
lo_frame_id <- base_frame_id
```

The transform from the sensor's frame to the odometry frame, which is the equivalent of what many stacks call `odom`.

### Amelioration mode

An estimate arrives, and the node reports where the sensor actually was.

**The matcher casts into the map at every pose it tries.** The map is the reference the matcher works against, throughout rotation and translation, rather than a scan frozen at the pose the correction started from. This is where the package departs from [cbgl](https://github.com/cbgl/cbgl), which the ray casting is borrowed from: cbgl manufactures one scan at the starting estimate and matches against that ring of points. A ring taken at one pose is only an approximation of the map, correct where it was taken and less so at every pose the matcher then walks to. Casting into the grid itself removes that approximation. What comes back is a corrected pose rather than a displacement.

An estimate is what triggers a correction. A scan on its own causes nothing to happen: the newest scan is held until an estimate asks for it.

**A transform is required.** `robot_frame_id`, default `base_footprint`, is looked up against the scan's own frame, because an incoming estimate places the robot and the matcher needs to know where the sensor sits on it. The whole rotation is read, not its heading alone, so a sensor mounted upside down needs nothing set. Without that transform in the tree every correction is refused for want of the sensor's placement.

#### Subscribed topics

| Topic | Type | Utility |
| --- | --- | --- |
| `scan_topic` | `sensor_msgs/msg/LaserScan` | 2d scans of a whole turn or less, centred on the sensor's heading, are published here |
| `map_topic` | `nav_msgs/msg/OccupancyGrid` | the map to match against |
| `pose_estimate_input_topic` | `geometry_msgs/msg/PoseWithCovarianceStamped` | the estimates to correct. Each one arriving triggers a correction |

The map subscription uses transient local durability at a depth of one, so a map latched before this node started is delivered on subscription rather than waited for. A second map replaces the first, and a correction already running finishes against the map it started with.

#### Published topics

| Topic | Type | Utility |
| --- | --- | --- |
| `pose_estimate_topic` | `geometry_msgs/msg/PoseWithCovarianceStamped` | the corrected estimate, carrying the covariance of the estimate it corrected untouched |
| `map_scan_topic` | `sensor_msgs/msg/LaserScan` | optional. The scan the corrected pose would see, cast from the map along the scan's own rays. It lies over the scan the sensor did see ray for ray, whatever the sensor covers. Leave the setting empty to publish nothing |

#### Services offered

| Service | Utility |
| --- | --- |
| `fsm_localisation/start` | commences node functionality |
| `fsm_localisation/stop` | halts node functionality, node remains alive |
| `fsm_localisation/correct_once` | corrects the estimate the node already holds, once, and returns the outcome in its message |

`correct_once` answers from the estimate the node is already holding rather than waiting for a fresh one. Waiting would block the service and would leave the "no pose estimate has arrived" refusal with no way to happen.

#### Transforms published

```
global_frame_id <- estimate_frame_id <- correction_frame_id
```

Two frames that exist only to carry this node's answer, so that no frame in an existing tree gains a second parent. The first receives the estimate as it arrived; the second receives the correction alone, so anything wanting only how far the estimate was out reads that one and nothing else. Composing the two gives the corrected robot pose.

#### Why a correction is refused

Every refusal is returned as a sentence. Before it matches, the node refuses a correction for these reasons:

- the node holds no map, or no scan, yet
- the scan spans more than a whole turn, or is not centred on the sensor's heading, or its first angle is not a number
- the estimate names a frame other than the map's
- the estimate's position is not finite, or its orientation carries no heading
- the estimate is stamped before zero
- the sensor's placement on the robot cannot be read from the transform tree, or carries no heading

The matcher then refuses for six reasons:

- the scan is too short to match
- the scan carries no usable range
- the scan's aperture is not something a sensor can cover
- the estimate is not on free space, which covers both a position outside the map and one on a blocked cell
- every ray cast from the estimate came back at maximum range, so the estimate sees nothing of the map
- the match spent its whole recovery budget without settling

The last is deliberately conservative. A match that needed every recovery it was allowed is refused even where the last attempt happened to converge, because from outside the matcher those two cases cannot be told apart.

A cell no sensor has ever seen blocks a ray, whatever `occupancy_threshold` is set to: the matcher may not walk into territory the map says nothing about.

## Parameters

Two files under `config/`, one per mode: `params_odometry.yaml` and `params_amelioration.yaml`. Both are commented at length; what follows is the summary.

A parameter outside its permitted range is refused at start-up with a sentence naming the setting and the value it was given, and the process exits with one rather than aborting.

### Shared by both modes

| Parameter | Description | Default |
| --- | --- | --- |
| `mode` | `odometry` or `amelioration`. No default; the node refuses to start without it | none |
| `scan_topic` | where 2d scans are published | `/base_scan` |
| `pose_estimate_topic` | where this node's answer is published. The message type differs by mode | `<node name>/pose_estimate` |
| `global_frame_id` | the global frame, for example `map` | `map` |
| `base_frame_id` | the lidar sensor's own frame | `base_laser_link` |
| `size_scan` | how many rays a scan is matched at. `0` matches every ray the scan carries | 0 |
| `min_magnification_size` | base angular oversampling | 0 |
| `max_magnification_size` | maximum angular oversampling | 3 |
| `num_iterations` | greater displacement between the two inputs requires higher values | 2 |
| `xy_bound` | axis-wise radius for drawing a new initial position when a recovery is attempted | 0.2 |
| `t_bound` | angular radius for the same | pi/4 |
| `max_counter` | lower values decrease execution time | 200 |
| `max_recoveries` | lower values decrease execution time | 10 |
| `rng_seed` | `0` draws the recovery search from hardware entropy, as always. Any other value seeds its generators, see below | 0 |
| `ray_search` | `angular` or `windowed`. How each ray is matched to the wall it meets | `angular` |
| `enforce_early_gearup` | jump to maximum oversampling once the rotation criterion passes 0.99999 | false |
| `enforce_translation_bound` | abandon a candidate whose position wanders more than `xy_bound` plus 0.01 from where the translation stage started | false |
| `recover_on_failed_sift` | start a recovery when every candidate angle fails its trial translation, rather than carrying on with an angle that failed | false |
| `scan_qos_reliability` | `reliable` or `best_effort`. Sensor drivers commonly publish `best_effort`, and a mismatched subscription receives nothing at all | `reliable` |
| `scan_qos_depth` | subscription queue depth | 1 |

Frame ids carry no leading slash. tf2 rejects them.

The last three matcher switches are carried across from the scan-to-map matcher this package borrows from. All three are unconditional in that matcher and absent from the odometry one. The shipped amelioration configuration turns two of them on and leaves `enforce_translation_bound` off. On a recording from a real robot, the bound refused about a quarter of the frames whose estimate was up to 0.25 m out. It also let more wrong answers through than with the bound off. The shipped odometry configuration leaves all three off. They default to off, since a node given neither configuration file has no mode to infer them from. With `recover_on_failed_sift` on, a correction reaches the recovery path more often than before. A correction that needs a recovery can differ from one run to the next, see `rng_seed` below.

`size_scan` decides how many rays a scan is matched at. `0`, the default, matches every ray the scan carries: the sensor's own resolution, with nothing discarded. In odometry mode the first scan to arrive settles the size for the session, since two scans can only be matched against each other at one size, and a later scan of a different length is resampled to it rather than dropped. In amelioration mode it is settled afresh for every correction, since one correction is never compared against another.

Any other value reduces every scan to that many rays before matching, and refuses a scan that carries fewer. This is the setting to reach for when matching cannot keep up. Execution time rises faster than the ray count does, so halving the rays buys back more than half the time. If a match takes longer than the interval between its inputs the node says so, periodically, and names this setting.

`ray_search` picks between two ways of finding the wall each ray meets. `angular` offers each wall only to the rays that can reach it. It returns the nearest wall in front of every ray whatever shape the room is, and its execution time rises in step with `size_scan`. `windowed` narrows the search for each ray to the neighbourhood of the wall the previous ray met. It is what this algorithm shipped with, and it is kept so that a run can be compared against results published before `angular` existed. Its execution time rises with the square of `size_scan`, and where a room turns back on itself it can return a wall standing behind the nearest one.

Measured over 13908 matches drawn from a recorded dataset at `size_scan: 360`, `angular` completes a match in 17.9 ms against `windowed`'s 25.4 ms, median. The two disagree on 0.67 per cent of matches, by a median of 3.5 mm. At `size_scan: 1440` the ray casting alone is 3.2 times faster, and at 5760, 12.4 times.

`rng_seed` decides how the recovery search draws the poses it restarts a match from. `0`, the default, draws them from hardware entropy, as this algorithm has always done. Any other value seeds the generator of each executor thread with that value, once, at the first draw on that thread. The node runs several executor threads, and any of them may run a match or a correction. Thus which draw a recovery gets depends on the thread schedule. Two runs over the same input can still differ at a step that needed a recovery. A correction can settle in one run and be refused in another. On a recording from a real robot, three odometry runs at `rng_seed` 1 went over the same 2000 scans. The second and the third each differed from the first from step 79. A match that needs no recovery draws nothing, so the seed does not change it.

### Odometry mode only

| Parameter | Description | Default |
| --- | --- | --- |
| `initial_pose_topic` | optional. Where an initial pose estimate may be provided | `<node name>/initial_pose` |
| `path_estimate_topic` | where the whole trajectory estimate is published | `<node name>/path_estimate` |
| `lo_topic` | where the odometry estimate is published | `<node name>/lo` |
| `lo_frame_id` | the odometry frame | `lo` |

### Amelioration mode only

| Parameter | Description | Default |
| --- | --- | --- |
| `map_topic` | the map to match against | `/map` |
| `pose_estimate_input_topic` | the estimates to correct | `/amcl_pose` |
| `map_scan_topic` | optional. Where the scan cast from the map is published. Empty publishes nothing | `<node name>/map_scan` |
| `map_scan_method` | `ray_marching`, `bresenham` or `cddt`. Which algorithm casts a ray into the map | `ray_marching` |
| `occupancy_threshold` | a cell at or above this, on the nought to one hundred scale, is blocked | 65 |
| `robot_frame_id` | the robot's own frame, which is what an incoming estimate places | `base_footprint` |
| `estimate_frame_id` | the frame the estimate as it arrived is broadcast to | `<node name>/estimate` |
| `correction_frame_id` | the frame the correction alone is broadcast to | `<node name>/corrected` |
| `tf_lookup_timeout` | seconds to wait for the sensor's placement to appear in the transform tree before giving up on one correction | 0.1 |

One caster is built per map, not three. `ray_marching` walks the true line and is the default. `bresenham` walks a lattice approximation of it, which costs accuracy on a ray meeting a wall at a shallow angle. `cddt` precomputes a table, which costs time and memory when a map arrives and quantises the cast heading to half a degree thereafter, which at thirty metres is about thirteen centimetres of lateral error, worse than a five centimetre map.

`map_scan_method` is checked where the map is built rather than at start-up. The three names are recognised in one place, because copying the list into a second place is how the two drift apart.

The sensor's frame used for a correction is read off the scan message rather than from `base_frame_id`, so nothing has to be told twice what the scan already says. `base_frame_id` keeps meaning the sensor everywhere.

## What the map costs and what it bounds

Measured rather than estimated, on a 1.70 GHz laptop core.

**A correction is cheaper than an odometry match, not dearer.** Over the same room, the same scan, the same settings and the same starting estimate, with only the reference changed:

| reference | one whole match | against the ring |
| --- | --- | --- |
| the previous scan, as a ring of points | 24.3 ms | |
| a map at 0.05 m a cell | 16.0 ms | 0.66 times |
| a map at 0.20 m a cell | 19.2 ms | 0.79 times |

One cast alone is cheaper still, between two and a half and three and a half times, at every ray count from 180 to 5760. A ray marched through a distance transform takes a handful of steps across open space, where a ray offered to a ring of points must be resolved against a wall list whose length is the ray count. The cost also barely moves between the two resolutions, sixteen times apart in cell count, because marching takes steps proportional to distance rather than to cells. A finer map is not a slower one.

**What the map costs is accuracy, and the cell size is the bound.** On the same comparison, recovering a pose whose truth is known:

| reference | position error |
| --- | --- |
| the previous scan, as a ring of points | 0.41 mm |
| a map at 0.05 m a cell | 3.19 mm |
| a map at 0.20 m a cell | 13.77 mm |

About a fifteenth of a cell either way. The ring of points carries the room to full double precision and the map carries it to the nearest cell, so the map's resolution sets the floor under a correction's accuracy. Choose it accordingly: this is the trade, and it is not the execution time.

## The borrowed ray casting

The three casters under `include/fsm_localisation/range_libc/` are `range_libc` by Corey H. Walsh, **Apache 2.0**, governed by the licence file beside them rather than by this package's MIT terms. They were taken from [cbgl](https://github.com/cbgl/cbgl)'s vendored copy rather than from `range_libc` directly. `distance_transform.h` is a separate work by Giorgio Marcias, carried along by `range_libc`, and is unmodified.

Removed on the way in, none of it used here: the graphics card ray marching path, its own benchmark harness, a fourth caster built on a giant precomputed lookup table, the trace map debugging feature, the file based grid constructors that needed a PNG decoder, and the decoder itself.

Four defects in cbgl's additions were corrected, each characterised by a test that failed for a stated reason before the correction was made. The full record, including the one place where the code deliberately follows cbgl rather than the published `range_libc` and the measurement that decided it, is in `include/fsm_localisation/range_libc/README.md`.

That the casting came across faithfully is measured, not asserted. Given the same grid and the same rays, this package's copy and cbgl's own answer identically on every one of 705024 casts across six grids, three casters and two ray counts.

## Diagnostics

The matching core reports in plain strings and does not write to a terminal itself. It hands each line to whatever destination the host installed, through `fsm_localisation::setDiagnosticSink`, and drops the line where nothing was installed. The node installs a destination that forwards to `RCLCPP_INFO`, so anything the core says arrives in the ordinary ROS log.

In an ordinary build there is almost nothing to say. The stage timings, which are the bulk of it, are compiled out unless the core is built with `FSM_LOCALISATION_TRACE`:

```sh
colcon build --cmake-args "-DCMAKE_CXX_FLAGS=-DFSM_LOCALISATION_TRACE"
```

That build is for finding out where the time goes and is not the one to run a robot with: it reads the clock around every stage of every iteration.

## Coming from fsm_lidar_odometry

The odometry mode **is** [`fsm_lidar_odometry`](https://github.com/fourier-scan-matcher/fsm-lidar-odometry), taken from commit `d343895c73cb535b40a758b3f86a6cfee01574a5` and renamed. The matcher is not merely equivalent, it is the same code: every step of this work that touched the core compared disassembly before and after, and where anything moved, ran all 97 match steps of three recorded scenarios through both builds at full hexadecimal precision.

What a caller has to change:

- **The package is `fsm_localisation`.** The node, the executable, the launch file, the include directory, the C++ namespace and the default node name all followed.
- **`mode` must be set.** There is no default. Anything launching this node must give it, which in practice means one of the two parameter files.
- **`config/params.yaml` is now two files**, `params_odometry.yaml` and `params_amelioration.yaml`, keyed by the wildcard `/**:` rather than by node name.
- **The four service names changed**, because they are node relative and therefore follow the node's name. `fsm_lidar_odometry/start`, `/stop`, `/clear_estimated_trajectory` and `/set_initial_pose` are now `fsm_localisation/start` and so on. Their types and their behaviour are unchanged.
- **The default topic names changed**, because they are built from the node's own name. What was `fsm_lidar_odometry/pose_estimate` is now `fsm_localisation/pose_estimate`, and the same for `path_estimate`, `lo`, and `initial_pose`. `scan_topic` still defaults to `/base_scan`. Anything setting these explicitly is unaffected.
- **The matcher's entry point is `FSM::Match::fmt`.** It was `fmtdbh`. This matters only to code that drives the core directly rather than through the node.

The core gained one thing and nothing else: a reference handle, so that the matcher can be handed either a ring of points or an occupancy grid. The ring path through it is unchanged, which is what the comparisons above establish.

## Upgrading from the ROS 1 version

This concerns the odometry mode, which is the part that has a ROS 1 predecessor.

The matcher itself is unchanged, and this is measured rather than asserted: driven over identical synthetic scans, the two versions agree on every published quantity to better than 4e-15, six orders of magnitude inside the 1e-9 bar the comparison demands. Both sides are checked against themselves first, three runs each, so that the comparison rests on something reproducible.

It is also faster. Driven over the same synthetic scans on the same machine, compiled with the same optimisation settings and with the ROS layer taken out of the picture on both sides, a match takes a little under half the time it did: 18.3 ms against 45.6 ms, median over 3444 matches across six scenarios. Roughly half of that comes from a compiler fifteen years newer and from returning results rather than writing them through output pointers, and the rest from the new `angular` ray search.

What did change:

- The four services return `std_srvs/srv/Trigger` instead of taking and returning nothing, so callers now get a success flag and a message.
- Output is stamped from the incoming scan rather than from the clock at the moment of processing. The reported twist follows from the interval between scan stamps, so it no longer varies with machine load.
- `std_msgs/Header` has no sequence number in ROS 2, so the published messages no longer carry one.
- The scan subscription's reliability and depth are parameters. The default mirrors ROS 1, but a `best_effort` sensor driver now needs `scan_qos_reliability` set to match, or no scans arrive at all.
- Frame id defaults lost their leading slash.
- A scan carrying fewer ranges than `size_scan` is refused with a warning instead of being read past its end.

Eight defects were corrected before the port, and the numbers above are measured against a ROS 1 build carrying those corrections rather than against the published one. Output from this version therefore differs from the published ROS 1 version by more than the figures above. The corrections:

- The rotation matrix that accumulates the trajectory mixed single and double precision, so it was not a rotation. Its determinant was 1.7e-8 away from unity, and since it is composed once per scan the error grew along the whole path.
- The initial pose supplied through `set_initial_pose` was built in single precision, degrading it to about seven significant figures.
- The convergence criterion of the translation stage took a single precision square root of a double.
- Gap filling read one element before the start of an empty list whenever a scan contained no invalid returns at all, which crashed the node. Real sensors nearly always return at least one bad ray, which is why this survived so long.
- With `pose_estimate_topic` unset, the fallback was written to the wrong field: the pose publisher was given an empty topic name and the node threw during construction, so it could not start at all without that parameter.
- Frame id fallbacks carried a leading slash, which tf2 rejects.
- Two assertions checked that an unsigned value was at least zero.
- Output was stamped from the wall clock rather than from the scan.

Two more were found and corrected in this package's own core: an out of bounds read in the rotation stage, and a read of an empty vector reachable through `recover_on_failed_sift`. The first changes the odometry path's reported rotation criterion, which the recorded scenarios could not have caught because they compare poses.

Known gaps and deliberate omissions are recorded in [`FOLLOWUP.md`](FOLLOWUP.md).

## Motivation and Under the hood

### 1 min summary video

[![IMAGE ALT TEXT](http://img.youtube.com/vi/hB4qsHCEXGI/0.jpg)](http://www.youtube.com/watch?v=hB4qsHCEXGI "1 min summary video")

### IROS 2022 paper

```bibtex
@INPROCEEDINGS{9981228,
  author={Filotheou, Alexandros and Sergiadis, Georgios D. and Dimitriou, Antonis G.},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={FSM: Correspondenceless scan-matching of panoramic 2D range scans},
  year={2022},
  pages={6968-6975},
  doi={10.1109/IROS47612.2022.9981228}}
```
