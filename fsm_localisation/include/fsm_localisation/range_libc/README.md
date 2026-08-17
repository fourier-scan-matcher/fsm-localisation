# range_libc

Borrowed code. Everything in this directory came from somewhere else and is governed by `LICENSE` beside it, not by this package's MIT terms.

## Where it came from

The ray marching, Bresenham's line and CDDT ray casters are range_libc, by Corey H. Walsh, Apache 2.0 licensed. They were taken from cbgl's vendored copy rather than from range_libc directly, at `github.com/cbgl/cbgl` commit `49bf5456cc02de0bfac9cef2a4568629ff22d3c1`, under `cbgl/include/utils/range_libc/`. The published range_libc that cbgl itself copied from is `github.com/kctess5/range_libc`, branch `deploy`, commit `1251dc3`, and it is the authority wherever cbgl's copy and it disagree.

`distance_transform.h` is a separate work by Giorgio Marcias, carried by range_libc as a third party dependency and kept here with its own copyright header intact. It is unmodified.

## What was left out

The graphics card ray marching path, its own benchmark harness, a fourth ray caster built on a giant precomputed lookup table that nothing here will ever construct, the trace map debugging feature, the file based grid constructors that needed a PNG decoder, and the decoder itself. None of it is used by this package.

One helper that only ever served the PNG path, `utils::rgb2gray`, is still here and is now unreachable. It also carries a defect of its own: its red coefficient is 0.229 where it should be 0.299, so its three coefficients sum to 0.93 and white reads as 237 rather than 255. Left rather than removed, so that a comparison against the source it came from stays readable.

## What was changed

The three casters' algorithms are cbgl's as written, character for character, with one exception, and it is the fourth correction below.

Four defects in cbgl's additions were corrected here. Each was characterised by a test that failed for a stated reason before the correction was made; those tests are in this package's own test directory.

1. **The in memory grid constructor left `width` and `height` unassigned.** Every read of them was a read of an indeterminate value, which produced a wrong size, a failed allocation or a crash depending on the run. That constructor is gone, replaced by `OMap(int, int, const std::vector<bool>&)`, which takes the dimensions and the cells and assigns both.
2. **It left its error flag unassigned too**, so a caller could not tell a sound grid from a broken one. Gone with the constructor. The grid now arrives already decoded and its size is checked before this file is reached.
3. **It left the six fields of the library's own world to grid conversion unassigned**, so a correct conversion was fed indeterminate values. Both surviving constructors now assign them to a defined identity: scale one, angle zero, origin zero, sine zero, cosine one. This package does not use that conversion, converting between world and cells itself, but no build of this file can now read a value nothing wrote. For anyone who does want it: `world_angle` is the negation of the map's yaw, not the yaw.
4. **`RayMarching::calc_range` rounded one axis down and the other up**, with a sign aware version commented out beside it under the word "backup" and no reason given for either. The effect is that a sensor standing in one row walks along the next, and a ray landing on a cell boundary comes out a cell wrong. Corrected to the published range_libc's plain truncation, and the commented out pair removed with it, since leaving a third rounding in the file as a comment is how the defect arose.

**One part of that fourth correction deliberately follows cbgl rather than the published original, and is the one place in this file where that is true.** Which cell a ray has reached and how far the sensor has travelled to reach it are two separate questions. This package now answers the first the way the published original does, truncation on both axes, and the second the way cbgl does, measuring to the point the ray actually reached rather than to the corner of the truncated cell. The published original's own answer to the second question is a defect in its own right: it discards a fraction of a cell that is correlated with the ray's own heading, so it does not average out and reads back as a fixed offset in the sensor's own position, which is exactly what this matcher exists to estimate. Measured over a bordered room, 720 rays from an off centre pose, the corner measurement's offset comes out at close to half a cell diagonally: about 3.6 cm on a five centimetre map and 14.2 cm on a twenty centimetre map, the second being worse than the map's own resolution. Measuring to the point the ray actually reached instead brings that down to about a millimetre and four millimetres respectively, some thirty five times smaller. The general rule behind every other correction in this file is that the published original is sound and cbgl's additions are where the defects live; here the reverse is true, cbgl's own answer is the more accurate one, and following the general rule would mean knowingly shipping a worse position estimate.

A fifth defect is not corrected here. `CDDTCast::calc_range` indexes the grid with no bounds test, so a pose outside the grid is undefined behaviour rather than a maximum range. It is guarded at this package's own boundary instead, in `occupancy_grid_map.cpp`, where a pose outside the grid returns the maximum range for every ray without reaching the caster at all. That guard is load bearing and must not be removed as redundant.

The remainder of the changes are what a compiler fifteen years newer than the code required, and none of them alters a value: signed against unsigned comparison repairs, two initialiser lists reordered to match declaration order, unnamed parameters on an unused virtual stub, one computed but unused variable marked as such, and three locally scoped warning suppressions, each carrying its reason above it.

## A switch that would not build

`_USE_LRU_CACHE` is defined to 0 at the head of `RangeLib.h`, and the CDDT caster carries a number of uses of that cache behind it. The header the cache lives in was not brought across, and the include that would reach it still carries cbgl's own directory layout. Turning that switch on therefore meets a missing header rather than a working cache. Turning it on would also make `calc_range` write to a shared member, which would end this package's guarantee that a map is safe to query from several threads at once; `occupancy_grid_map.cpp` holds that switch and one other at zero with a static assertion for exactly that reason.
