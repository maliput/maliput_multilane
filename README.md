
[![gcc](https://github.com/maliput/maliput_multilane/actions/workflows/build.yml/badge.svg)](https://github.com/maliput/maliput_multilane/actions/workflows/build.yml)

# maliput_multilane

## Description

`maliput_multilane` is an implementation of [Maliput](https://github.com/maliput/maliput)'s API that allows users to
instantiate a road network with the following relevant characteristics:

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

**Note**: For full information about Maliput please visit [Maliput Documentation](https://maliput.readthedocs.io/en/latest/index.html).

### Resources

`maliput_multilane` provides several map resources at [maliput_multilane/resources](resources).
These resources are YAML files that contains a description for the road geometry creation.
Some of them, also describe `maliput`'s road network information like: Range Value Rules, Discrete Value Rules, Traffic Lights, Phase Rings, Intersections, etc.

Resources are installed natively, so the users are able to use them for their own interest.
In order to get the installation path check the environment variable: `MULTILANE_RESOURCE_ROOT`.

## API Documentation

Refer to [Maliput Multilane's Online API Documentation](https://maliput.readthedocs.io/en/latest/html/deps/maliput_multilane/html/index.html).

## Examples

[Getting Started](https://maliput.readthedocs.io/en/latest/getting_started.html) page is a good place for starting to see Maliput's capabilities and how to use a Maliput backend for getting a road network.

 - [maliput_integration](https://github.com/maliput/maliput_integration): Concentrates applications created for maliput. See [maliput_integration's tutorials](https://maliput.readthedocs.io/en/latest/html/deps/maliput_integration/html/integration_tutorials.html). These applications allow to select `maliput_multilane` as the backend.
 - [maliput_integration_tests](https://github.com/maliput/maliput_integration_tests): `maliput_multilane` is used extensively for the maliput integration tests.

## Installation

### Supported platforms

Ubuntu Focal Fossa 20.04 LTS.

### Binary Installation on Ubuntu

See [Installation Docs](https://maliput.readthedocs.io/en/latest/installation.html#binary-installation-on-ubuntu).

### Source Installation on Ubuntu

#### Prerequisites

```
sudo apt install python3-rosdep python3-colcon-common-extensions
```

#### Build

1. Create colcon workspace if you don't have one yet.
    ```sh
    mkdir colcon_ws/src -p
    ```

2. Clone this repository in the `src` folder
    ```sh
    cd colcon_ws/src
    git clone https://github.com/maliput/maliput_multilane.git
    ```

3. Install package dependencies via `rosdep`
    ```
    export ROS_DISTRO=foxy
    ```
    ```sh
    rosdep update
    rosdep install -i -y --rosdistro $ROS_DISTRO --from-paths src
    ```

4. Build the package
    ```sh
    colcon build --packages-up-to maliput_multilane
    ```

    **Note**: To build documentation a `-BUILD_DOCS` cmake flag is required:
    ```sh
    colcon build --packages-select maliput_multilane --cmake-args " -DBUILD_DOCS=On"
    ```
    More info at [Building Documentation](https://maliput.readthedocs.io/en/latest/developer_guidelines.html#building-the-documentation).

For further info refer to [Source Installation on Ubuntu](https://maliput.readthedocs.io/en/latest/installation.html#source-installation-on-ubuntu)


### For development

It is recommended to follow the guidelines for setting up a development workspace as described [here](https://maliput.readthedocs.io/en/latest/developer_setup.html).

## Contributing

Please see [CONTRIBUTING](https://maliput.readthedocs.io/en/latest/contributing.html) page.

## License

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://github.com/maliput/maliput_multilane/blob/main/LICENSE)
