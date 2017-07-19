from ros2run.api import get_executable_path


def launch(descriptor, argv):
    """Launch the slam gmapping node.

    * ld -- the launch descriptor object
    * argv -- the command line arguments

    """
    args = []

    # TODO: Eventually support input arguments

    slamGmappingExecutable = get_executable_path(
        package_name="gmapping",
        executable_name="slam_gmapping")

    descriptor.add_process([slamGmappingExecutable] + args)
