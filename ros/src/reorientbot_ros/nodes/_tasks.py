import pybullet_planning as pp
import trimesh

import mercury

from mercury.examples.reorientation import _utils


def set_obj_goal(self, obj_goal):
    self._obj_goal = obj_goal
    self._env.PLACE_POSE = pp.get_pose(self._obj_goal)
    c = mercury.geometry.Coordinate(*self._env.PLACE_POSE)
    c.translate([0, 0, 0.2], wrt="world")
    self._env.PRE_PLACE_POSE = c.pose
    # highlight target pose
    visual_file = pp.get_visual_data(self._obj_goal)[
        0
    ].meshAssetFileName.decode()
    mesh = trimesh.load(visual_file)
    mesh.apply_transform(
        mercury.geometry.transformation_matrix(*self._env.PLACE_POSE)
    )
    pp.draw_aabb(mesh.bounds, color=(1, 0, 0, 1))


def task_01(self):
    # bin
    obj = mercury.pybullet.create_bin(
        X=0.3, Y=0.3, Z=0.11, color=(0.7, 0.7, 0.7, 1)
    )
    pp.set_pose(
        obj,
        (
            (0.4495000000000015, 0.5397000000000006, 0.059400000000000126),
            (0.0, 0.0, 0.0, 1.0),
        ),
    )
    self._env.bg_objects.append(obj)

    # target place
    obj = mercury.pybullet.create_mesh_body(
        visual_file=mercury.datasets.ycb.get_visual_file(class_id=3),
        quaternion=_utils.get_canonical_quaternion(class_id=3),
        rgba_color=(1, 1, 1, 0.5),
        mesh_scale=(0.99, 0.99, 0.99),  # for virtual rendering
    )
    pp.set_pose(
        obj,
        (
            (0.44410000000000166, 0.5560999999999995, 0.02929999999999988),
            (
                -0.5032839784369476,
                -0.4819772480647679,
                -0.4778992452799924,
                0.5348041517765217,
            ),
        ),
    )
    set_obj_goal(self, obj)


def task_02(self):
    if 0:
        self._subscriber_base.subscribe()
        while self._subscriber_base_points is None:
            pass
        self._subscriber_base.unsubscribe()

    # bin
    obj = mercury.pybullet.create_bin(
        X=0.3, Y=0.3, Z=0.11, color=(0.7, 0.7, 0.7, 1)
    )
    mercury.pybullet.set_pose(
        obj,
        (
            (0.5105999999999966, -0.004099999999999998, 0.08820000000000094),
            (0.0, 0.0, -0.003999989333334819, 0.9999920000106667),
        ),
    )
    if 0:
        mercury.pybullet.annotate_pose(obj)
    self._env.bg_objects.append(obj)

    parent = obj

    # cracker_box
    obj = mercury.pybullet.create_mesh_body(
        mercury.datasets.ycb.get_visual_file(class_id=2)
    )
    mercury.pybullet.set_pose(
        obj,
        (
            (
                -0.03843769431114197,
                -0.06841924041509628,
                -0.020100004971027374,
            ),
            (
                0.6950822472572327,
                0.003310413332656026,
                0.7187591791152954,
                0.01532843615859747,
            ),
        ),
        parent=parent,
    )

    # sugar_box
    obj = mercury.pybullet.create_mesh_body(
        mercury.datasets.ycb.get_visual_file(class_id=3)
    )
    pp.set_pose(
        obj,
        (
            (
                0.41110000000000757,
                0.05680000000000025,
                0.05700000000000005,
            ),
            (
                0.6950242570144113,
                0.009565935485992997,
                0.7188680629825536,
                0.008859066742884331,
            ),
        ),
    )
    mercury.pybullet.set_pose(
        obj,
        (
            (
                -0.054823607206344604,
                0.06029653549194336,
                -0.031200002878904343,
            ),
            (
                0.6950822472572327,
                0.003310413332656026,
                0.7187591791152954,
                0.01532843615859747,
            ),
        ),
        parent=parent,
    )

    # mustard_bottle
    obj = mercury.pybullet.create_mesh_body(
        mercury.datasets.ycb.get_visual_file(class_id=5)
    )
    pp.set_pose(
        obj,
        (
            (0.569899999999996, -0.06540000000000118, 0.06160000000000018),
            (
                0.16129618279208996,
                0.6966397860813726,
                0.6862419415410084,
                -0.13322367483006758,
            ),
        ),
    )
    mercury.pybullet.set_pose(
        obj,
        (
            (0.1017511785030365, -0.06474190950393677, -0.026600003242492676),
            (
                0.16755934059619904,
                0.695159912109375,
                0.6874131560325623,
                -0.12704220414161682,
            ),
        ),
        parent=parent,
    )

    # tomato_can
    obj = mercury.pybullet.create_mesh_body(
        mercury.datasets.ycb.get_visual_file(class_id=4),
        rgba_color=(1, 1, 1, 0.5),
        mesh_scale=(0.99, 0.99, 0.99),  # for virtual rendering
    )
    pp.set_pose(
        obj,
        (
            (0.5252000000000009, 0.07049999999999995, 0.06700000000000034),
            (
                -0.03968261331907751,
                0.7070224261017329,
                0.7038029876137785,
                0.05662096621665727,
            ),
        ),
    )
    mercury.pybullet.set_pose(
        obj,
        (
            (0.059504538774490356, 0.07194063067436218, -0.02120000123977661),
            (
                -0.03331788629293442,
                0.7073509097099304,
                0.7032650113105774,
                0.06295282393693924,
            ),
        ),
        parent=parent,
    )
    set_obj_goal(self, obj)
