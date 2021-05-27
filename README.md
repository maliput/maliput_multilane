# `maliput_multilane`


Maliput_multilane is an implementation of Maliput's API that allows users to
instantiate a RoadGeometry with the following relevant characteristics:

- Multiple Lanes are allowed per Segment.
- Constant width Lanes.
- Segments with lateral asphalt extensions, aka shoulders.
- Line and Arc base geometries, composed with cubic elevation and superelevation
  polynomials.
- Semantic Builder API.
- YAML based map description.
- Adjustable linear tolerance.

The number of lanes and their lengths, widths, and shoulder widths are all user
specifiable.

## Build

1. Setup a development workspace as described [here](https://github.com/ToyotaResearchInstitute/maliput-documentation/blob/main/docs/installation_quickstart.rst).

2. Bring up your development workspace:

```sh
cd path/to/my/workspace
source ./bringup
```

3. Build `maliput_multilane` package and its dependencies:

  - If not building drake from source:

   ```sh
   colcon build --packages-up-to maliput_multilane
   ```

  - If building drake from source:

   ```sh
   colcon build --cmake-args -DWITH_PYTHON_VERSION=3 --packages-up-to maliput_multilane
   ```
