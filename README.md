# Noisy depth camera sensor for Gazebo

Gazebo normally doesn't add noise to depth camera sensor images. 
Even though the SDF specification allows for doing so, Gazebo just ignores the noise tag.

This plugin adds support for reading this noise tag and even allows for specifying other noise implementations.

## Implemented noise models

- `gaussian`: The standard additive gaussian noise. Not very good for depth images.
- `gaussian_multiplicative`: Multiplicative gaussian noise. A bit better for depth images.

## Installation

This is a "pure" Gazebo plugin (i.e. not gazebo_ros plugin), however, it needs ROS to work.
That is because there's no nice support in Gazebo for releasing custom sensor models.
To get the model in Gazebo, we use the [gazebo_custom_sensor_preloader](https://github.com/peci1/gazebo_custom_sensor_preloader)
system plugin.

So, to get this plugin working, add this repo and gazebo_custom_sensor_plugin into a catkin workspace
and build it. Then you have to create a custom gazebo launcher that will add `libgazebo_custom_sensor_preloader.so` to
gzserver commandline before launching.

For quick testing, just build the workspace, source it, and call

    rosrun gazebo_ros gzserver -s libgazebo_custom_sensor_preloader.so my.world

Any depth camera in your world will now read the specified noise.

## Performance

The noise is added to the image using a CPU loop, not on the GPU as RGB noise.

Unfortunately, I wasn't able to utilize OpenGL for adding the noise.
I tried hard, but using an analogous approach that's used for RGB noise,
all I could get were either all black or all grey images, nothing more.
I suppose the problem could be that the DepthCamera hard-sets the current pass
to the rendering system, so adding more compositors doesn't work very well.
PRs welcome to fix this! 