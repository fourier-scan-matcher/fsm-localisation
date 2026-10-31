# Known gaps and deferred work

Recorded while this package was built, first during the port of the odometry
mode from ROS 1 and then during the merge that added the amelioration mode.
None of these blocks the package; all were left alone deliberately, and the
reason is given so that the decision can be revisited rather than rediscovered.

Table of contents
=================

* [The borrowed ray casting](#the-borrowed-ray-casting)
* [The map and the amelioration mode](#the-map-and-the-amelioration-mode)
* [The matcher core](#the-matcher-core)
* [Parameters and the node](#parameters-and-the-node)
* [Build, packaging and containers](#build-packaging-and-containers)
* [Proofs and comparisons](#proofs-and-comparisons)
* [Choices carried across from the scan-to-map code](#choices-carried-across-from-the-scan-to-map-code)

## The borrowed ray casting

### Four defects in cbgl's grid reading, corrected, and a fifth guarded elsewhere

The three casters under `include/fsm_localisation/range_libc/` came from cbgl's
vendored copy of `range_libc`. cbgl had added a constructor that builds the
occupancy grid from an in-memory pixel buffer rather than from a PNG file, and
that addition was defective. Each defect was characterised by a test that failed
for a stated reason before anything was corrected; those tests are in this
package's test directory.

1. **The in-memory constructor never assigned the grid's `width` and `height`.**
   Both were read afterwards as indeterminate values, so depending on the run
   the result was a grid of the wrong size, a failed allocation, or a crash. In
   the benign case the grid came out empty, every ray reported maximum range,
   and every correction was made against a manufactured view of empty space.
2. **It tested an uninitialised error flag** before doing any work and returned
   early, leaving the grid empty, whenever that indeterminate value happened to
   be non-zero.
3. **It left the six fields of the library's own world to grid conversion
   unassigned**, so a correct conversion was being fed indeterminate values.
4. **Ray marching chose its cell by rounding one axis down and the other up**,
   with a sign aware version commented out beside it under the word "backup" and
   no reason given for either. The effect is that a sensor standing in one row
   walks along the next, and a ray landing on a cell boundary comes out a cell
   wrong.

The first two are gone with the constructor, which was replaced by one taking
the dimensions and a flat vector of cells. The third is assigned to a defined
identity in both surviving constructors. The fourth was corrected to the
published `range_libc`'s plain truncation.

A fifth is **not** corrected in that file. `CDDTCast::calc_range` indexes the
grid with no bounds test, so a pose outside the grid is undefined behaviour
rather than a maximum range. It is guarded at this package's own boundary
instead, in `occupancy_grid_map.cpp`, where a pose outside the grid returns the
maximum range for every ray without reaching the caster at all. **That guard is
load bearing and must not be removed as redundant.**

The full record, including the one place where this package deliberately follows
cbgl rather than the published `range_libc` and the measurement that decided it,
is in `include/fsm_localisation/range_libc/README.md`.

### cbgl's file constructor is broken too, and is not corrected here

Beyond the four above, cbgl's PNG file constructor calls the in-memory one as a
plain statement rather than delegating to it, so it constructs a temporary,
throws it away, and never fills the grid whatever the file holds. Every path
into cbgl's occupancy grid was therefore broken.

Nothing here uses it: the file constructors were not carried across, because
they needed the PNG decoder and this package builds its grid from a ROS message.
Recorded because anyone comparing against cbgl, or reading cbgl, will meet it.

### Two warning suppressions in the borrowed header

`-Wtype-limits` at `RangeLib.h:1107` and `-Wmaybe-uninitialized` at
`RangeLib.h:1265`, both locally scoped and both carrying their reason above them.
The first is a guard whose negative half is always false because the value is
unsigned, and whose other half already rejects the same case. The second is a
false positive on `calc_range_pair`, which nothing here calls.

They are suppressed rather than fixed because the corrections made to that file
are the ones its defects forced, and a warning on a path nothing takes is not one
of them. Anyone re-syncing the library against a newer upstream should read them
again rather than carry them forward on trust.

### `utils::rgb2gray` has the wrong red coefficient

Its red coefficient is 0.229 where it should be 0.299, so its three coefficients
sum to 0.93 and white reads as 237 rather than 255.

The function is now unreachable: it only ever served the PNG path, which was not
carried across. It is left in place so that a comparison against the source it
came from stays readable. Anyone reinstating a PNG path must fix it first.

### `_USE_LRU_CACHE` cannot be turned on

It is defined to 0 at the head of `RangeLib.h`, and the CDDT caster carries a
number of uses of that cache behind it. The header the cache lives in was not
brought across, and the include that would reach it still carries cbgl's own
directory layout, so turning the switch on meets a missing header rather than a
working cache.

Turning it on would also make `calc_range` write to a member shared across
threads, which would end this package's guarantee that one map is safe to query
from several threads at once. `occupancy_grid_map.cpp` holds that switch and
`_TRACK_COLLISION_INDEXES` at zero with a static assertion for exactly that
reason.

### CDDT quantises the cast heading to half a degree

720 bins, matching cbgl so that a comparison against it compares like with like.
That is about thirteen centimetres of lateral error at thirty metres, which is
worse than a five centimetre map. Worth knowing before `cddt` is chosen for its
speed. `ray_marching`, the default, has no such quantisation.

The bin count is also written twice across a module boundary: once in
`occupancy_grid_map.cpp` where the caster is built, and once in the tests where
it sets every tolerance. Two places that must agree and nothing making them.

## The map and the amelioration mode

### Whether a cell nobody has seen should block a ray is a live question

It blocks, and that is what the specification asks for: the matcher may not walk
into territory the map says nothing about. It is not a parameter.

The choice is recorded as open rather than settled because the argument
originally offered in its favour turned out to be factually wrong. The
consequence of blocking is that a map with unsurveyed interior gives shorter
ranges than the room really has; the consequence of passing through is that a
ray leaves the surveyed area and reports whatever it meets beyond it, or nothing.
Which is right depends on how the maps in use are produced.

### The scan's `range_max` is not screened

It is taken from the incoming message and handed to the caster untouched. A
value that is not a number, or is not positive, is screened nowhere.

Not in the specification, and noticed while writing the corrector. Left rather
than inventing a refusal, because what a node should do with a sensor reporting a
nonsensical maximum range is a question about sensors and not about this
matcher. Every other field of the message the corrector reads is screened.

### A quaternion whose norm is infinite is accepted where the map is built

The map builder refuses an origin orientation whose components are not finite,
and refuses one whose norm squared is below a floor. Four finite components can
still square-sum to infinity: the lower bound then passes, the normalisation
divides by infinity, and the yaw comes out exactly zero. A map with a rotated
origin would be silently read as unrotated.

One line, plus the test that would justify it.

### A wall sitting exactly on a cell boundary is not characterised

Truncation assigns a point on a cell boundary to the cell below. Where a wall
sits exactly on such a boundary, which cell the wall is in is decided by that
rule rather than by the geometry, and reflecting the grid about its horizontal
axis moves the answer by one cell. Measured: 52 rays in 34560 on a bordered room.

The synthetic generator sidesteps the case by nudging its grid origin off round
numbers. Real recorded maps contain walls on cell boundaries. What the package
does there is defined but has not been characterised, and the same one cell of
slack already exists between the three casters.

### The ray count contract is not checked where it is consumed

`FSM::GridReference::ranges` carries a written contract about the angle of ray
`i`, and the core's grid ray search reads the vector that comes back. Nothing
checks that the implementation returned the number of ranges it was asked for.
The base class is public and a test stub already derives from it, so a
non-conforming implementation is reachable.

## The matcher core

### An unmeasurable range is treated as no reading, but nothing is clamped

Infinity, not-a-number, zero and any negative range are all taken to mean the
sensor could not measure that ray, and are filled in from the rays either side.
A scan with nothing valid in it is refused.

What is still open is whether a ray that reports `range_max` exactly, which some
drivers use to mean "nothing within range" rather than "a surface at exactly this
distance", should be treated the same way. It currently is not. Deciding that
needs a survey of what drivers actually do.

### Twenty six core functions are shipped and exercised by nothing

The core header declares seventy three functions. The runtime path reaches forty
five. Of the twenty eight it does not, one is unreachable on purpose and one has
a test; the remaining twenty six are compiled into the package and run by nothing
at all.

They are listed and grouped, with the method used to arrive at the list, in the
header comment of `fsm_localisation/test/test_core_golden.cpp`.

They are listed rather than deleted because deleting them is a decision about how
much of the original algorithm this package is obliged to carry, and that is not
a decision to take while porting or merging.

### `Rotation::fmt2Sequential` is unreachable

`Match::fmt` passes the literal `"batch"` at its one call site, so the sequential
branch of `Rotation::fmt` can never be taken. The function is a complete
alternative implementation of the rotation stage that nothing can reach.

Either make the choice a parameter or delete the branch. Leaving it as a literal
means the code reads as though there were a choice when there is not.

### A scan built from any harmonic but the first carries no translation information

Found while building a test fixture, and it is a property of the algorithm rather
than a defect. The translation stage reads only the first harmonic of the
difference between the scan it was given and the scan it would see. A synthetic
scan whose shape is built from the second harmonic or above therefore yields no
translation at all, whatever its amplitude.

This matters to anyone writing a test fixture: a room built from a higher
harmonic will look like a translation failure and is not one.

### The intersection of an oblique ray and a wall is good to about 1e-9

A ray is represented by a point a hundred million metres along it, and where that
ray meets a wall perpendicular to the x-axis the height is worked out by
subtracting two quantities of that size. Almost all of it cancels, and about
eight of the sixteen digits go with it.

The remaining accuracy is a few parts in a thousand million, which is the same
order as the tolerance the cross version comparison is held to. It does not
threaten that comparison, both versions computing it the same way, but it does
bound how accurately any single scan can be matched, and it is not obvious from
reading the code.

The fix is to compute the height from the ray's own gradient rather than from the
far point, which is one line and changes the numbers.

### Gap filling never returns for a scan in which nothing was measured

Given a scan whose every reading is missing, the gap filling appends a list of
indices to itself while walking that same list, and allocates until the process
is killed.

Nothing reaches it: the matcher refuses a scan with nothing valid in it before
gap filling is called, and `test_scan_handling.cpp` holds that guard in place. The
function on its own is still unsafe for any other caller.

### The iteration counters in the matcher's output are always zero

`output_params::translation_iterations` and `rotation_iterations` are only
accumulated under `FSM_LOCALISATION_TRACE`. In every shipping build they read
zero, and nothing says so at the point of use.

`num_recoveries`, the rotation criterion and the translation criterion are
always populated, so anything wanting to know how hard a match worked should read
those. Either populate the counters unconditionally, which costs two increments,
or move them behind a type that cannot be read by mistake.

### A guard against a value that is not finite can be folded away

The package builds at `-Ofast -fno-finite-math-only`. `-Ofast` alone implies
`-ffinite-math-only`, under which the compiler is entitled to assume no infinity
or not-a-number ever exists: a guard written to catch one folds to nothing, and
an equality comparison against a literal can return true for a value that is not
a number. That single sub-flag caused a whole class of defect in this family, and
disabling it costs a parity check on the rare double comparison the hot loops
make while keeping every other relaxation `-Ofast` brings.

The remedy inside the code is to read the exponent bits rather than call
`std::isfinite` or compare against a literal, as `isValidRange` in
`fsm_localisation.cpp` and the two helpers in `occupancy_grid_map.cpp` do.
Whether a given comparison folds is not predictable across translation units:
the same comparison with the same flags gave opposite answers in a probe and in a
real target.

`test_fsm_localisation_finite_guard` pins its own flags with `SHELL:` and fails to
compile if the pin is lost. **Do not append compile options without `SHELL:`**:
CMake de-duplicates against directory scope and silently reorders, which turned
finite math back on in a target written to defeat it.

### `isValidRange` is exercised at exact settings and never as it ships

It used to be reached through the shared library, whose arithmetic follows the
build option rather than the test's own. The suites that force exact arithmetic
now compile it themselves, so what they measure is what they asked for.

What no test reaches is the function as the package ships it. The one target
compiled that way, `test_fsm_localisation_finite_guard`, deliberately links
nothing, and giving it the function would mean a second object library at the
shipped settings. Since the remedy inside the function is to read the exponent
bits, which no arithmetic flag can fold, this is a gap in coverage rather than a
suspected defect.

### The core header is duplicated in the `fsm` repository

`include/fsm_localisation/fsm_core.hpp` and `include/fsm.h` in
[`fsm`](https://github.com/fourier-scan-matcher/fsm) are two copies of the same
algorithm that have drifted apart. Differences found while porting and merging:

- `fsm` carries measurement noise parameters and a terminal constraint that this
  copy does not.
- This copy carries the transform accumulation, scan subsampling, gap filling and
  reference handle that the ROS wrapper and the map path need, which `fsm` does
  not.
- One index variable that selects the best candidate has a different type in each
  copy, which can select a different candidate.

Only this copy received the ten corrections listed in the readme, and only this
copy has the angular ray search and the occupancy grid path. The `fsm` copy still
carries the same two matcher defects this package corrected, at `include/fsm.h`
around line 3593 and in four calls that fold away under finite math.

Porting the two matcher fixes across needs `std::tuple` poses and pointer outputs
rather than a `Pose` struct, and a finite check without `std::bit_cast` since that
copy is C++11. It is header only and needs `csm`. Reconciling the two is worth
doing and is a job in itself. It was explicitly out of scope.

## Parameters and the node

### A negative integer parameter wraps instead of being refused

`size_scan: -1` and four others are read as signed and cast to unsigned, so they
wrap to enormous values. The node starts happily and then refuses every scan,
with nothing pointing at the parameter file. `scan_qos_depth: -1` throws from the
middleware rather than from a check, so the message names the middleware and not
the setting.

No parameter in this package carries a descriptor with a declared range, which is
what would catch this declaratively and would also make the range visible to
`ros2 param describe`. Found, recorded, not fixed: it is a small amount of work
per parameter and the node declares thirty three.

### Odometry mode assumes a sensor that sees all the way round

Amelioration mode reads what a scan covers, and refuses a scan that spans more than a whole turn or is not centred on the sensor's heading. Odometry mode does not read it. It lays every scan out over a whole turn, so the rays of a narrower sensor are matched at angles they were not taken at. Nothing refuses such a scan with a sentence that explains why. The core carries a scan completion routine that would fill in a partial scan. Nothing calls it.

## Build, packaging and containers

### The copyright header of the exported cmake template is no longer checked

The exported cmake file is now a template that records the arithmetic this
package was built with, so it is `cmake/fsm_localisation-extras.cmake.in` rather
than `cmake/fsm_localisation-extras.cmake`. The cmake style checker still reads
it under the new name. The copyright checker does not: it recognises the old
extension and not the new one, so the file count it reports fell from 37 to 36
and the licence header of that one file is now unchecked.

The header is present and unchanged. What is lost is the automated check on it,
one file's worth. Extending the copyright checker's file set means switching off
the automatic discovery for that one checker and listing the files by hand,
which is more machinery than the gap is worth. Recorded rather than repaired.


### The two formatting linters are switched off

`ament_cmake_uncrustify` and `ament_cmake_cpplint` impose a brace and wrapping
style this package has never used. Satisfying them means reformatting several
thousand lines of inherited numerical code, which is the riskiest change
available for no behavioural gain. Every other check `ament_lint_common`
provides is on.

### A consumer's own arithmetic settings govern the matcher it links

The package now exports targets a downstream package can link, and nearly all of
the matcher is in the header, so a consumer compiles those functions itself, at
its own flags, and its own copies are the ones its calls reach. The figures this
package publishes were measured at the settings its own `CMakeLists.txt` sets and
nothing characterises what tolerance holds at any other.

The alternative is to export the arithmetic settings as a usage requirement of
the exported targets, which imposes them on a consumer that may have its own
reasons for the settings it chose. Left to the readme to warn about instead.

### The repository the manifest names does not exist yet

`package.xml`, `CITATION.cff` and this package's own readme all name
`fourier-scan-matcher/fsm-localisation`, which returns 404. The organisation
exists and hosts the predecessor, so the name is the intended home rather than a
mistake, but it has to be created before any of those three tell the truth. There
is no apt package and no published image either, and the readme no longer claims
otherwise.

### A third copy of the personal shell profile lives in the proof tree

`docker/bashrc_base` was an eleven hundred line personal shell configuration,
carried in from the machine of whoever built the first image in this family and
shipped to anyone who opened a shell in the image. It has been replaced by fifty
lines that this package owns, and the predecessor's copy replaced with the same
fifty.

`fsm-localisation-tests/lyrical/docker/bashrc_base` is still the original. That
image is a development container rather than a shipped one, and its ROS half
differs from this package's: it reads `$USER`, which its own Dockerfile sets as
an environment variable, where the two package containers read `${HOME}`. The
three files cannot simply be made one without settling that difference first.

### The image build has two network fetches and neither retries

`rosdep update` and the lookup that finds the current ROS apt source release both
reach the network, and each is a bare command in its own layer. Either failing
fails the build, with a Python traceback rather than a sentence.

They are normally invisible because the layers cache. Clearing the build cache
and rebuilding twice in an afternoon is enough to be rate limited by
`raw.githubusercontent.com`, which answers 429, which rosdep reports as a read
timeout. Nothing about the package is wrong when that happens and nothing in the
output says so.

There were three. `rosdep init` is gone: the base image already ships the sources
list it writes, byte for byte, so deleting that file and fetching an identical
copy cost a round trip and bought nothing. The two that remain genuinely need the
network. A retry with a short backoff around each would close it, and is left
because the failure is loud rather than silent.

The image has since been rebuilt with every container change this package
carries, and everything they claimed holds in it: the sources list the base
image ships is present, byte for byte the file the deleted step used to fetch;
a shell opens on a fifty six line profile with the workspace on its path,
`colcon_cd` a function and no compiler exported; and the two shared directories
are still owned by the host user after the container has mounted and written to
them.

### The container cannot be told to run the node

Its main process is a shell. The entrypoint drops from root to the container's
user and execs whatever it was given, without sourcing the ROS environment
first, so giving the compose file a `command` that runs the node would meet
`ros2: command not found`. The environment comes from the user's shell profile
and from nowhere else.

The consequence is that `docker compose up` starts a container sitting in a shell
rather than a running node, and the node has to be launched into it afterwards.
The readme says so and gives the two arguments an `exec` needs. Making the
container run the node on its own means sourcing the environment in the
entrypoint, which is a small change to a file inherited verbatim from an
upstream project and was left for a decision about what the shipped container is
for.

## Proofs and comparisons

### The comparison harness does not assert that recovery stayed quiet

Because the recovery search draws its restart from hardware entropy, a comparison
run in which it fired would be meaningless, and the harness is supposed to fail
rather than report a difference it cannot explain. It does not check.

The node reports the count truthfully and logs a warning whenever a match needed
recovery, so the evidence exists in the run output; no warning appears in any of
the six scenarios. What is missing is the harness reading it and failing on it,
on both sides. Closing it properly means adding the same reporting to the ROS 1
reference build, which means rebuilding a pinned container and regenerating every
reference recording, disturbing a proof that currently passes.

The same gap has now been seen to bite in the other direction: the replay of the
published experiment moved three mean pose errors in one setting, entirely
because of the single match in ten thousand eight hundred that used the recovery
search. The comparator sets such a match aside from its disagreement columns and
not from its mean columns.

### The recovery search cannot be compared across versions at all

The standard library does not specify how a distribution turns random bits into a
number, so two implementations may differ even from the same seed. Comparing the
recovery path across versions would first need that conversion written out
explicitly rather than taken from the library.

A non-zero `rng_seed` does not make a run repeat, even within one build. Each executor thread seeds its own generator, and any of them may run a match. Thus which draw a recovery gets depends on the thread schedule. A unit test shows only that one thread seeded alike draws alike.

The seed also reached only one branch of the predecessor's proof tree. The other still seeds from hardware entropy, with no setting to change that. That changes nothing about the reference recordings, which were captured from a build where recovery never fired.

### Casting is not integer arithmetic, and the specification assumed it was

Two of the three casters answer with a floating point distance rather than a
count of cells: ray marching returns the distance to the point the ray reached,
and CDDT compares floating point positions inside a lookup table. Only
Bresenham's line is pure integer traversal.

The consequence is that two compilers do not agree on their last bits. Measured:
at worst three hundredths of a thousandth of a cell, about 1.5 micrometres on a
five centimetre map, on about forty per cent of ray marching casts. It is far too
small to matter to a robot and it is fatal to any comparison that assumes exact
agreement across builds. Anyone writing such a comparison must hold the compiler
and its flags equal on both sides.

## Choices carried across from the scan-to-map code

Five behaviours were found in the scan-to-map matcher that the odometry core did
not have. Three came across as parameters, all defaulting off, so the odometry
mode is unchanged: `enforce_early_gearup`, `enforce_translation_bound` and
`recover_on_failed_sift`. The last two exist in the scan-to-map code only and not
in the `fsm` repository's copy.

### The terminal constraint and the two noise parameters were dropped

The terminal constraint declares a match bad when the translation criterion
exceeds a multiple of the combined sensor and map noise. The two noise standard
deviations were the only thing it read, and neither they nor it appear anywhere
in this package.

Anyone reinstating it must first settle a discrepancy found alongside it: the
multiplier is 4 in the scan-to-map code and 2 in the `fsm` repository's copy, and
nothing says which is intended.

### The scan-to-map copy holds a candidate index as a `double`

The index that selects the best candidate is a `double` there where both other
copies hold it as an unsigned integer. That is a defect in that copy and is not
ported. Recorded so that nobody ports it later by matching the wrong reference.

### The panel's commit guard and pre-commit requirement are not adopted

The agent panel this work was executed with prescribes a `commit-guard` skill and
a pre-commit hook. Neither exists in this repository and neither repository in
this family carries a pre-commit configuration. Noted rather than silently
ignored.
