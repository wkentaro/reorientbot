import path
import skrobot

import reorientbot


class Panda(skrobot.models.Panda):
    def __init__(self, *args, **kwargs):
        root_dir = path.Path(reorientbot.__file__).parent
        urdf_file = (
            root_dir / "_pybullet/data/franka_panda/panda_drl.urdf"
        )  # NOQA
        super().__init__(urdf_file=urdf_file)

    @property
    def rarm(self):
        link_names = ["panda_link{}".format(i) for i in range(1, 8)]
        links = [getattr(self, n) for n in link_names]
        joints = [link.joint for link in links]
        model = skrobot.model.RobotModel(link_list=links, joint_list=joints)
        model.end_coords = self.tipLink
        return model


def main():
    import IPython
    import numpy as np

    viewer = skrobot.viewers.TrimeshSceneViewer()

    robot_model = Panda()
    viewer.add(robot_model)
    viewer.add(skrobot.model.Box((2, 2, 0), vertex_colors=(0.7, 0.7, 0.7)))

    viewer.set_camera(
        angles=[np.deg2rad(80), 0, np.deg2rad(60)],
        distance=2,
        center=(0, 0, 0.5),
    )

    viewer.show()
    IPython.embed()


if __name__ == "__main__":
    main()
