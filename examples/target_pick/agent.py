import os
import queue

import numpy as np
import torch

from yarr.agents.agent import ActResult
from yarr.agents.agent import Agent
from yarr.agents.agent import ScalarSummary

from pose_net import PoseNet


class DqnModel(torch.nn.Module):
    def __init__(self, env):
        super().__init__()

        self.module = PoseNet(n_action=len(env.actions))

    def forward(self, observation):
        ee_pose = observation["ee_pose"]

        # grasp_pose = observation["grasp_pose"]
        # grasp_position = observation["grasp_pose"][:, :3]
        # past_actions = observation["past_actions"]
        # past_actions = past_actions.reshape(past_actions.shape[0], -1)

        object_labels = observation["object_labels"]
        object_poses = observation["object_poses"]
        grasp_flags = observation["grasp_flags"]

        return self.module(
            ee_pose=ee_pose,
            grasp_flags=grasp_flags,
            object_labels=object_labels,
            object_poses=object_poses,
        )


class DqnAgent(Agent):
    def __init__(self, env):
        self._env = env
        self._epsilon = np.nan
        self._losses = queue.deque(maxlen=18)

    def build(self, training, device=None):
        self.q = DqnModel(env=self._env).to(device).train(training)
        if training:
            self.q_target = DqnModel(env=self._env).to(device).train(False)
            for p in self.q_target.parameters():
                p.requires_grad = False
            self.optimizer = torch.optim.Adam(self.q.parameters(), lr=1e-3)
        else:
            for p in self.q.parameters():
                p.requires_grad = False

    def act(self, step, observation, deterministic, env):
        obs = {}
        for key in observation:
            obs[key] = torch.as_tensor(observation[key][:, 0])
            assert obs[key].shape[0] == 1

        with torch.no_grad():
            q = self.q(obs)
            q = q.numpy()
        assert q.shape[0] == 1
        actions_select = np.argsort(q[0])[::-1]

        if deterministic:
            for a in actions_select:
                act_result = ActResult(action=a)
                j = env.validate_action(act_result)
                if j is not None:
                    act_result.j = j
                    break
            else:
                act_result = ActResult(actions_select[0])
        else:
            self._epsilon = epsilon = self._get_epsilon(step)
            if np.random.random() < epsilon:
                for a in np.random.permutation(q.shape[1]):
                    act_result = ActResult(action=a)
                    j = env.validate_action(act_result)
                    if j is not None:
                        act_result.j = j
                        break
            else:
                for a in actions_select:
                    act_result = ActResult(action=a)
                    j = env.validate_action(act_result)
                    if j is not None:
                        act_result.j = j
                        break
                else:
                    act_result = ActResult(actions_select[0])
        return act_result

    def _get_epsilon(self, step):
        epsilon_init = 1
        epsilon_final = 0.01
        min_step = 0
        max_step = 5000
        if max_step == min_step:
            alpha = 1
        else:
            alpha = min(1, max(0, (step - min_step) / (max_step - min_step)))
        epsilon = alpha * epsilon_final + (1 - alpha) * epsilon_init
        return epsilon

    def _get_sampling_weights(self, replay_sample):
        loss_weights = 1.0
        if "sampling_probabilities" in replay_sample:
            self.indicies = replay_sample["indices"]
            probs = replay_sample["sampling_probabilities"]
            loss_weights = 1.0 / torch.sqrt(probs + 1e-10)
            loss_weights /= torch.max(loss_weights)
        return loss_weights

    def update(self, step, replay_sample):
        self._update_q(replay_sample)
        soft_updates(self.q, self.q_target, tau=0.005)
        return dict(priority=self._priority)

    def _update_q(self, replay_sample):
        action = replay_sample["action"].long()
        reward = replay_sample["reward"]

        terminal = replay_sample["terminal"].float()

        def stack_timesteps(x):
            return torch.cat(torch.split(x, 1, dim=1), -1).squeeze(1)

        obs = {}
        obs_tp1 = {}
        for key in replay_sample:
            if key in [
                "action",
                "reward",
                "terminal",
                "timeout",
                "indices",
                "sampling_probabilities",
            ]:
                continue
            if key.endswith("_tp1"):
                obs_tp1[key[:-4]] = stack_timesteps(replay_sample[key])
            else:
                obs[key] = stack_timesteps(replay_sample[key])

        with torch.no_grad():
            qs_target = self.q_target(obs_tp1)
            q_target = torch.max(qs_target, dim=1).values
            q_target = reward + 0.99 * (1 - terminal) * q_target

        qs_pred = self.q(obs)

        q_pred = qs_pred[torch.arange(qs_pred.shape[0]), action]

        self.optimizer.zero_grad()

        sampling_weigths = self._get_sampling_weights(replay_sample)
        q_delta = torch.nn.functional.smooth_l1_loss(
            q_pred, q_target, reduction="none"
        )
        q_loss = (q_delta * sampling_weigths).mean()

        loss = q_loss

        loss.backward()

        self.optimizer.step()

        self._losses.append(loss.item())
        self._priority = torch.sqrt(q_delta + 1e-10).detach()

    def update_summaries(self):
        return [
            ScalarSummary("agent/loss", np.mean(self._losses)),
        ]

    def act_summaries(self):
        return [ScalarSummary("agent/epsilon", self._epsilon)]

    def load_weights(self, save_dir):
        device = torch.device("cpu")
        self.q.load_state_dict(
            torch.load(os.path.join(save_dir, "q.pth"), map_location=device)
        )

    def save_weights(self, save_dir):
        torch.save(self.q.state_dict(), os.path.join(save_dir, "q.pth"))


def soft_updates(net, target_net, tau):
    for param, target_param in zip(net.parameters(), target_net.parameters()):
        target_param.data.copy_(
            tau * param.data + (1 - tau) * target_param.data
        )
