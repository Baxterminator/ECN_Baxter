from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    # Show RViz
    sl.include("baxter_description", "rviz_launch.py")

    # Start the bridge
    sl.node("baxter_bridge", "bridge", arguments=["--server"])

    with sl.group(ns="baxter"):
        # Launch setup part
        with sl.group(ns="setup"):
            sl.node("ecn_baxter", "setup_server")

        # Launch Game Master Node
        with sl.group(ns="game_master"):
            sl.node("ecn_baxter", "game_master")
    return sl.launch_description()
