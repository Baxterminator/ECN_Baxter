from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    with sl.group(ns="game_master"):
        sl.node("ecn_baxter", "setup_node")
        sl.node("ecn_baxter", "game_master")
        sl.node("image_orchestra", "image_composer")

    return sl.launch_description()
