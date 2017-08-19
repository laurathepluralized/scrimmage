"""Provide OpenAI interface for SCRIMMAGE."""
from __future__ import print_function
import threading
import subprocess
import time
import os
import signal
import sys
from pprint import pprint

import numpy as np
import queue
import grpc
from concurrent import futures
import gym
import gym.spaces
import google.protobuf.empty_pb2
import lvdb

from .proto import ExternalControl_pb2_grpc


class ServerThread(threading.Thread):
    """Start SCRIMMAGE ExternalControl GRPC Service."""

    def __init__(self, queues, max_workers=10, address="localhost:50051"):
        """Initialize variables."""
        super(ServerThread, self).__init__()
        self.queues = queues
        self.address = address
        self.max_workers = max_workers

    def run(self):
        """Start SCRIMMAGE ExternalControl GRPC Service."""
        server = grpc.server(
            futures.ThreadPoolExecutor(max_workers=self.max_workers))

        ExternalControl_pb2_grpc.add_ExternalControlServicer_to_server(
            ExternalControl(self.queues), server)
        server.add_insecure_port(self.address)
        server.start()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            server.stop(0)


class ScrimmageEnv(gym.Env):
    """OpenAI implementation for SCRIMMAGE."""

    def __init__(self):
        """Create queues for multi-threading."""
        self.queues = \
            {s: queue.Queue() for s in ['env', 'action', 'action_response']}

        files = [os.path.join(p, '..', 'missions', 'external_control.xml')
                 for p in os.environ['SCRIMMAGE_DATA_PATH'].split(':') if p]
        self.mission_file = next((f for f in files if os.path.isfile(f)))

        ServerThread(self.queues).start()

        self.scrimmage_process = \
            subprocess.Popen(["scrimmage", self.mission_file])

        environment = self.queues['env'].get()

        self.action_space = \
            _create_tuple_space(environment.action_spaces)
        self.observation_space = \
            _create_tuple_space(environment.observation_spaces)
        self.reward_range = (environment.min_reward, environment.max_reward)

        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Exit cleanly <ctrl-c> (i.e., kill the subprocesses)."""
        print("exiting scrimmage...")
        self.scrimmage_process.kill()
        sys.exit(0)

    def _reset(self):
        """Restart scrimmage and return result."""
        self.scrimmage_process.kill()
        self.scrimmage_process = \
            subprocess.Popen(["scrimmage", self.mission_file])
        return self._return_action_result()

    def _step(self, action):
        """Send action to SCRIMMAGE and return result."""
        self.queues['action_response'].put(action)
        return self._return_action_result()

    def _return_action_result(self):
        res = self.queues['action_response'].get()
        size_discrete = self.observation_space.spaces[0].num_discrete_space
        print("1", res)
        discrete_obs = np.array(res.observations.value[:size_discrete])
        print("2", discrete_obs)
        continuous_obs = np.array(res.observations.value[size_discrete:])
        print("3", continuous_obs)
        obs = tuple((discrete_obs, continuous_obs))
        print("4", obs)
        info = {}
        print("5")
        return obs, res.reward, res.done, info


class ExternalControl(ExternalControl_pb2_grpc.ExternalControlServicer):
    """GRPC Service to communicate with SCRIMMAGE Autonomy."""

    def __init__(self, queues):
        """Receive queues for multi-threading."""
        self.queues = queues

    def SendEnvironment(self, env, context):
        """Receive Environment proto and send back an action."""
        self.queues['env'].put(env)
        return google.protobuf.empty_pb2.Empty()

    def SendActionResult(self, action_result, context):
        """Receive ActionResult proto and send back an action."""
        self.queues['action_response'].put(action_result)
        return self.queues['action'].get()


def _create_tuple_space(space_params):
    discrete_extrema = []
    continuous_extrema = []

    def _append(param, dst_lst):
        if param.num_dims != 1 and len(param.maximum) == 1:
            dst_lst += param.num_dims * [param.minimum, param.maximum]
        else:
            dst_lst += zip(list(param.minimum), list(param.maximum))

    for param in space_params.params:
        if param.discrete:
            _append(param, discrete_extrema)
        else:
            _append(param, continuous_extrema)

    discrete_space = gym.spaces.MultiDiscrete(discrete_extrema)

    low, high = zip(*continuous_extrema)
    continuous_space = gym.spaces.Box(np.array(low), np.array(high))
    return gym.spaces.Tuple((discrete_space, continuous_space))
