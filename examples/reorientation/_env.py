import copy
import pickle
import time

from loguru import logger
import numpy as np
import path
import pybullet as p
import pybullet_planning as pp

import mercury

from _get_heightmap import get_heightmap
import _utils


home = path.Path("~").expanduser()


class Env:

    # parameters
    IMAGE_HEIGHT = 240
    IMAGE_WIDTH = 320

    HEIGHTMAP_PIXEL_SIZE = 0.004
    HEIGHTMAP_IMAGE_SIZE = 128
    HEIGHTMAP_SIZE = HEIGHTMAP_PIXEL_SIZE * HEIGHTMAP_IMAGE_SIZE

    TABLE_OFFSET = 0.01

    PILES_DIR = home / "data/mercury/pile_generation"
    PILE_TRAIN_IDS = np.arange(0, 1000)
    PILE_EVAL_IDS = np.arange(1000, 1200)
    PILE_POSITION = np.array([0.5, 0, TABLE_OFFSET])

    CAMERA_POSITION = np.array([PILE_POSITION[0], PILE_POSITION[1], 0.7])

    def __init__(
        self,
        class_ids,
        gui=True,
        retime=1,
        step_callback=None,
        mp4=None,
        face="front",
        real=False,
        robot_model="franka_panda/panda_suction",
        debug=True,
    ):
        super().__init__()

        self._class_ids = class_ids
        self._gui = gui
        self._retime = retime
        self._step_callback = step_callback
        self._mp4 = mp4
        self._face = face
        self._real = real
        self._robot_model = robot_model

        self.debug = debug
        self.eval = False
        self.random_state = np.random.RandomState()

    def shutdown(self):
        pp.disconnect()

    def launch(self):
        pp.connect(use_gui=self._gui, mp4=self._mp4)
        pp.add_data_path()

    def reset(self, pile_file=None):
        if not pp.is_connected():
            self.launch()

        pp.reset_simulation()
        pp.enable_gravity()
        pp.set_camera_pose((0.7, -1, 0.8), (0, 0.1, 0.3))
        p.configureDebugVisualizer(
            p.COV_ENABLE_SHADOWS, True, lightPosition=(100, -100, 0.5)
        )
        with pp.LockRenderer():
            self.plane = pp.load_pybullet("plane.urdf")
            pp.set_texture(self.plane)
            pp.set_color(self.plane, (0.8, 0.8, 0.8))
            pp.set_pose(self.plane, ([0, 0, self.TABLE_OFFSET], [0, 0, 0, 1]))

            z = 0.002 + self.TABLE_OFFSET
            for x in np.linspace(-5, 5, num=21):
                pp.add_line((x, -10, z), (x, 10, z), color=(0.5, 0.5, 0.5, 1))
            for y in np.linspace(-5, 5, num=21):
                pp.add_line((-10, y, z), (10, y, z), color=(0.5, 0.5, 0.5, 1))

            self._walls = []
            if self._real:
                wall = pp.create_box(
                    w=3, l=0.01, h=1.05, color=(0.6, 0.6, 0.6, 1)
                )
                pp.set_pose(wall, ([0, 0.6, 1.05 / 2], [0, 0, 0, 1]))
                self._walls.append(wall)
                wall = pp.create_box(
                    w=0.01, l=3, h=1.05, color=(0.7, 0.7, 0.7, 1)
                )
                pp.set_pose(wall, ([-0.4, 0, 1.05 / 2], [0, 0, 0, 1]))
                self._walls.append(wall)
                wall = pp.create_box(w=3, l=3, h=0.5, color=(1, 1, 1, 1))
                pp.set_pose(wall, ([0, 0, 0.25 + 1.05], [0, 0, 0, 1]))
                self._walls.append(wall)

        self.ri = mercury.pybullet.PandaRobotInterface(
            suction_max_force=None,
            suction_surface_threshold=np.inf,
            suction_surface_alignment=False,
            planner="RRTConnect",
            robot_model=self._robot_model,
        )
        self.ri.add_camera(
            pose=([-0.065, 0.058, -0.062], [0.003, -0.032, -0.009, 0.999]),
            fovy=np.deg2rad(54),
            height=self.IMAGE_HEIGHT,
            width=self.IMAGE_WIDTH,
        )

        if self._real:
            self.object_ids = None
            self.fg_object_id = None
            self.PLACE_POSE = None
            self.LAST_PRE_PLACE_POSE = None
            self.PRE_PLACE_POSE = None
            self._shelf = -1
        else:
            raise_on_error = pile_file is not None

            if pile_file is None:
                if self.eval:
                    i = self.random_state.choice(self.PILE_EVAL_IDS)
                else:
                    i = self.random_state.choice(self.PILE_TRAIN_IDS)
                pile_file = self.PILES_DIR / f"{i:08d}.pkl"

            with open(pile_file, "rb") as f:
                data = pickle.load(f)

            PILE_AABB = (
                self.PILE_POSITION + [-0.25, -0.25, -0.05],
                self.PILE_POSITION + [0.25, 0.25, 0.50],
            )
            # pp.draw_aabb(PILE_AABB)

            num_instances = len(data["class_id"])
            object_ids = []
            fg_object_ids = []
            for i in range(num_instances):
                class_id = data["class_id"][i]
                position = data["position"][i]
                quaternion = data["quaternion"][i]

                position += self.PILE_POSITION

                visual_file = mercury.datasets.ycb.get_visual_file(
                    class_id=class_id
                )
                collision_file = mercury.pybullet.get_collision_file(
                    visual_file
                )

                class_name = mercury.datasets.ycb.class_names[class_id]
                visibility = data["visibility"][i]
                logger.info(
                    f"class_id={class_id:02d}, "
                    f"class_name={class_name}, "
                    f"visibility={visibility:.1%}"
                )

                with pp.LockRenderer():
                    object_id = mercury.pybullet.create_mesh_body(
                        visual_file=visual_file,
                        collision_file=collision_file,
                        mass=mercury.datasets.ycb.masses[class_id],
                        position=position,
                        quaternion=quaternion,
                    )
                pp.set_dynamics(object_id, lateralFriction=0.7)

                contained = pp.aabb_contains_aabb(
                    pp.get_aabb(object_id), PILE_AABB
                )
                if not contained:
                    pp.remove_body(object_id)
                    continue

                object_ids.append(object_id)
                if class_id in self._class_ids and visibility > 0.95:
                    fg_object_ids.append(object_id)

            if not fg_object_ids:
                if raise_on_error:
                    raise RuntimeError
                else:
                    return self.reset()

            self.object_ids = object_ids
            self.fg_object_id = self.random_state.choice(fg_object_ids)

            pp.draw_aabb(
                pp.get_aabb(self.fg_object_id),
                color=(1, 0, 0),
                width=2,
            )

            self._shelf, self.PLACE_POSE = _utils.init_place_scene(
                env=self,
                class_id=_utils.get_class_id(self.fg_object_id),
                random_state=copy.deepcopy(self.random_state),
                face=self._face,
            )
            self.LAST_PRE_PLACE_POSE = self.PLACE_POSE
            c = mercury.geometry.Coordinate(*self._place_pose)
            c.translate([0, -0.3, 0], wrt="world")
            self.PRE_PLACE_POSE = c.pose

            for _ in range(int(1 / pp.get_time_step())):
                p.stepSimulation()
                if self._step_callback:
                    self._step_callback()
                if self._gui:
                    time.sleep(pp.get_time_step())

            self.setj_to_camera_pose()
            self.update_obs()

        self.bg_objects = [self.plane, self._shelf] + self._walls

    def setj_to_camera_pose(self):
        self.ri.setj(self.ri.homej)
        j = None
        while j is None:
            c = mercury.geometry.Coordinate(*self.ri.get_pose("camera_link"))
            c.position = self.CAMERA_POSITION
            j = self.ri.solve_ik(
                c.pose, move_target=self.ri.robot_model.camera_link
            )
        self.ri.setj(j)

    def update_obs(self):
        rgb, depth, segm = self.ri.get_camera_image()
        # if pp.has_gui():
        #     import imgviz
        #
        #     imgviz.io.cv_imshow(
        #         np.hstack((rgb, imgviz.depth2rgb(depth))), "update_obs"
        #     )
        #     imgviz.io.cv_waitkey(100)
        fg_mask = segm == self.fg_object_id
        camera_to_world = self.ri.get_pose("camera_link")

        K = self.ri.get_opengl_intrinsic_matrix()
        pcd_in_camera = mercury.geometry.pointcloud_from_depth(
            depth, fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2]
        )
        pcd_in_world = mercury.geometry.transform_points(
            pcd_in_camera,
            mercury.geometry.transformation_matrix(*camera_to_world),
        )

        aabb = np.array(
            [
                self.PILE_POSITION - self.HEIGHTMAP_SIZE / 2,
                self.PILE_POSITION + self.HEIGHTMAP_SIZE / 2,
            ]
        )
        aabb[0][2] = 0
        aabb[1][2] = 0.5
        _, _, segmmap, pointmap = get_heightmap(
            points=pcd_in_world,
            colors=rgb,
            ids=segm,
            aabb=aabb,
            pixel_size=self.HEIGHTMAP_PIXEL_SIZE,
        )

        self.obs = dict(
            rgb=rgb,
            depth=depth,
            fg_mask=fg_mask.astype(np.uint8),
            segm=segm,
            K=self.ri.get_opengl_intrinsic_matrix(),
            target_instance_id=self.fg_object_id,
            segmmap=segmmap,
            pointmap=pointmap,
            camera_to_world=np.hstack(camera_to_world),
        )


def main():
    env = Env(class_ids=[2, 3, 5, 11, 12, 15])
    while True:
        env.reset()
        mercury.pybullet.pause()


if __name__ == "__main__":
    main()