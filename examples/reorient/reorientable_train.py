#!/usr/bin/env python

import argparse
import collections
import datetime
import json
import pickle
import time

import numpy as np
import path
import pytz
import torch
from torch.utils.tensorboard import SummaryWriter
import tqdm

import mercury


home = path.Path("~").expanduser()
here = path.Path(__file__).abspath().parent


class ConvEncoder(torch.nn.Module):
    def __init__(self, out_channels):
        super().__init__()

        # heightmap: 1
        in_channels = 1
        self.encoder = torch.nn.Sequential(
            torch.nn.Conv2d(
                in_channels, 4, kernel_size=3, stride=1, padding=1
            ),
            torch.nn.ReLU(),
            torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0),
            torch.nn.Conv2d(4, 8, kernel_size=3, stride=1, padding=1),
            torch.nn.ReLU(),
            torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0),
            torch.nn.Conv2d(8, 16, kernel_size=3, stride=1, padding=1),
            torch.nn.ReLU(),
            torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0),
            torch.nn.Conv2d(16, 24, kernel_size=3, stride=1, padding=1),
            torch.nn.ReLU(),
            torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0),
            torch.nn.Conv2d(24, 32, kernel_size=3, stride=1, padding=1),
            torch.nn.ReLU(),
            torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0),
            torch.nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1),
            torch.nn.ReLU(),
            torch.nn.AvgPool2d(4, stride=4),
        )

        # h: 64
        # fg_object_label: 7
        # fg_object_pose: 7
        # grasp_pose: 6
        # reorient_pose: 7
        reorient_pose: 7
        in_channels = 64 + 7 + 7 + 6 + 7
        self.mlp = torch.nn.Sequential(
            torch.nn.Linear(in_channels, out_channels),
            torch.nn.ReLU(),
            torch.nn.Linear(out_channels, out_channels),
            torch.nn.ReLU(),
            torch.nn.Linear(out_channels, out_channels),
            torch.nn.ReLU(),
        )

    def forward(
        self,
        heightmap,
        object_label,
        object_pose,
        grasp_pose,
        reorient_pose,
    ):
        B, _, H, W = heightmap.shape
        _, A, _ = grasp_pose.shape

        h_obs = self.encoder(heightmap)
        h_obs = h_obs.reshape(B, h_obs.shape[1])

        h_obs = torch.cat([h_obs, object_label, object_pose], dim=1)

        h_obs = h_obs[:, None, :].repeat(1, A, 1)
        h = torch.cat([h_obs, grasp_pose, reorient_pose], dim=2)
        h = self.mlp(h)

        return h


class Model(torch.nn.Module):
    def __init__(self):
        super().__init__()

        self.encoder = ConvEncoder(128)
        self.fc_reorientable = torch.nn.Sequential(
            torch.nn.Linear(128, 3),
            torch.nn.Sigmoid(),
        )
        self.fc_trajectory_length = torch.nn.Sequential(
            torch.nn.Linear(128, 1),
        )

    def forward(
        self,
        heightmap,
        object_label,
        object_pose,
        grasp_pose,
        reorient_pose,
    ):
        h = self.encoder(
            heightmap=heightmap,
            object_label=object_label,
            object_pose=object_pose,
            grasp_pose=grasp_pose,
            reorient_pose=reorient_pose,
        )
        reorientable = self.fc_reorientable(h)
        trajectory_length = self.fc_trajectory_length(h)[:, :, 0]

        return reorientable, trajectory_length


class Dataset(torch.utils.data.Dataset):

    ROOT_DIR = home / "data/mercury/reorient/reorientable"

    def __init__(self, split):
        self._split = split

        self._files = {"train": [], "val": []}
        for i in range(0, 4000):
            seed_dir = self.ROOT_DIR / f"s-{i:08d}"
            if not seed_dir.exists():
                continue
            for pkl_file in sorted(seed_dir.walk("*.pkl")):
                self._files["train"].append(pkl_file)
        for i in range(4000, 5000):
            seed_dir = self.ROOT_DIR / f"s-{i:08d}"
            if not seed_dir.exists():
                continue
            for pkl_file in sorted(seed_dir.walk("*.pkl")):
                self._files["val"].append(pkl_file)

        for key, value in self._files.items():
            print(f"{key}: {len(value)}")

    def __len__(self):
        return len(self._files[self._split])

    def __getitem__(self, i):
        file = self._files[self._split][i]
        with open(file, "rb") as f:
            data = pickle.load(f)

        class_ids = [2, 3, 5, 11, 12, 15, 16]
        object_labels = []
        for object_class in data["object_classes"]:
            object_label = np.zeros(7)
            object_label[class_ids.index(object_class)] = 1
            object_labels.append(object_label)
        object_labels = np.array(object_labels, dtype=np.int32)

        object_fg_flags = data["object_fg_flags"]
        object_poses = data["object_poses"]

        object_label = object_labels[object_fg_flags == 1][0]
        object_pose = object_poses[object_fg_flags == 1][0]

        ee_to_obj = np.hsplit(data["grasp_pose_wrt_obj"], [3])
        grasp_point_start = ee_to_obj[0]
        grasp_point_end = mercury.geometry.transform_points(
            [[0, 0, 1]], mercury.geometry.transformation_matrix(*ee_to_obj)
        )[0]
        grasp_pose = np.hstack([grasp_point_start, grasp_point_end])

        reorient_pose = data["reorient_pose"]
        reorientable = np.r_[
            data["graspable"], data["placable"], data["reorientable"]
        ]
        trajectory_length = np.array(data["trajectory_length"])

        heightmap = data["pointmap"][:, :, 2][None, :, :]

        return dict(
            heightmap=heightmap,
            object_label=object_label,
            object_pose=object_pose,
            grasp_pose=grasp_pose[None],
            reorient_pose=reorient_pose[None],
            reorientable=reorientable[None],
            trajectory_length=trajectory_length[None],
        )


def epoch_loop(
    epoch,
    is_training,
    model,
    data_loader,
    summary_writer,
    optimizer=None,
    lambda1=1,
    lambda2=1,
):
    if is_training:
        assert optimizer is not None

    train_or_val = "train" if is_training else "val"

    model.train(is_training)

    classes_pred = []
    classes_true = []
    losses = collections.defaultdict(list)
    for iteration, batch in enumerate(
        tqdm.tqdm(
            data_loader,
            desc=f"{train_or_val.capitalize()} loop",
            leave=False,
            ncols=100,
        )
    ):
        reorientable_pred, trajectory_length_pred = model(
            heightmap=batch["heightmap"].float().cuda(),
            object_label=batch["object_label"].float().cuda(),
            object_pose=batch["object_pose"].float().cuda(),
            grasp_pose=batch["grasp_pose"].float().cuda(),
            reorient_pose=batch["reorient_pose"].float().cuda(),
        )
        reorientable_true = batch["reorientable"].float().cuda()
        trajectory_length_true = batch["trajectory_length"].float().cuda()

        reorientable_pred = reorientable_pred[:, 0, :]
        reorientable_true = reorientable_true[:, 0, :]
        trajectory_length_pred = trajectory_length_pred[:, 0]
        trajectory_length_true = trajectory_length_true[:, 0]

        if is_training:
            optimizer.zero_grad()

        classes_true.extend(
            (reorientable_true > 0.5).cpu().numpy().all(axis=1).tolist()
        )
        classes_pred.extend(
            (reorientable_pred > 0.5).cpu().numpy().all(axis=1).tolist()
        )

        loss_reorientable = torch.nn.functional.binary_cross_entropy(
            reorientable_pred, reorientable_true
        )
        loss_trajectory_length = torch.nn.functional.smooth_l1_loss(
            trajectory_length_pred[reorientable_true[:, 2] == 1],
            trajectory_length_true[reorientable_true[:, 2] == 1],
        )
        loss = lambda1 * loss_reorientable + lambda2 * loss_trajectory_length
        losses["loss_reorientable"].append(loss_reorientable.item())
        losses["loss_trajectory_length"].append(loss_trajectory_length.item())
        losses["loss"].append(loss.item())

        if is_training:
            loss.backward()
            optimizer.step()

            for name, values in losses.items():
                summary_writer.add_scalar(
                    f"train/{name}",
                    values[-1],
                    global_step=len(data_loader) * epoch + iteration,
                    walltime=time.time(),
                )
    classes_pred = np.array(classes_pred)
    classes_true = np.array(classes_true)

    metrics = dict()
    if not is_training:
        for name, values in losses.items():
            metrics[name] = np.mean(values)
        tp = (classes_true & classes_pred).sum()
        fp = (~classes_true & classes_pred).sum()
        tn = (~classes_true & ~classes_pred).sum()
        fn = (classes_true & ~classes_pred).sum()
        metrics["accuracy"] = (tp + tn) / (tp + fp + tn + fn)
        metrics["precision"] = tp / (tp + fp)
        metrics["recall"] = tp / (tp + fn)
        metrics["specificity"] = tn / (tn + fp)
        metrics["balanced"] = (metrics["recall"] + metrics["specificity"]) / 2
        metrics["f1"] = 2 / (1 / metrics["precision"] + 1 / metrics["recall"])
        for key, value in metrics.items():
            summary_writer.add_scalar(
                f"val/{key}",
                value,
                global_step=len(data_loader) * epoch + iteration,
                walltime=time.time(),
            )

    return metrics


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--name", required=True, help="name")
    parser.add_argument("--lambda1", type=float, default=1, help="lambda1")
    parser.add_argument("--lambda2", type=float, default=1, help="lambda2")
    args = parser.parse_args()

    data_train = Dataset(split="train")
    loader_train = torch.utils.data.DataLoader(
        data_train, batch_size=256, shuffle=True, drop_last=True
    )
    data_val = Dataset(split="val")
    loader_val = torch.utils.data.DataLoader(
        data_val, batch_size=256, shuffle=False
    )

    model = Model()
    model.cuda()

    optimizer = torch.optim.Adam(params=model.parameters(), lr=1e-3)

    now = datetime.datetime.now(pytz.timezone("Asia/Tokyo"))
    log_dir = (
        here / "logs/reorientable" / now.strftime("%Y%m%d_%H%M%S.%f")
        + "-"
        + args.name
    )
    log_dir.makedirs_p()

    git_hash = mercury.utils.git_hash(cwd=here, log_dir=log_dir)
    with open(log_dir / "params.json", "w") as f:
        json.dump({"git_hash": git_hash}, f)

    writer = SummaryWriter(log_dir=log_dir)

    max_metric = (-1, -np.inf)
    with tqdm.trange(-1, 200, ncols=100) as pbar:
        for epoch in pbar:
            pbar.set_description(
                f"Epoch loop ({max_metric[1]:.3f} @{max_metric[0]})"
            )
            pbar.refresh()

            # train epoch
            if epoch >= 0:
                epoch_loop(
                    epoch=epoch,
                    is_training=True,
                    model=model,
                    data_loader=loader_train,
                    summary_writer=writer,
                    optimizer=optimizer,
                    lambda1=args.lambda1,
                    lambda2=args.lambda2,
                )

            # val epoch
            with torch.no_grad():
                metrics = epoch_loop(
                    epoch=epoch,
                    is_training=False,
                    model=model,
                    data_loader=loader_val,
                    summary_writer=writer,
                    optimizer=optimizer,
                    lambda1=args.lambda1,
                    lambda2=args.lambda2,
                )
            if epoch >= 0 and metrics["f1"] > max_metric[1]:
                model_file = (
                    log_dir / f"models/model_best-epoch_{epoch:04d}.pt"
                )
                model_file.parent.makedirs_p()
                torch.save(model.state_dict(), model_file)
                max_metric = (epoch, metrics["f1"])


if __name__ == "__main__":
    main()
