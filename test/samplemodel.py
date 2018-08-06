
def _get_action(i):
    return 1 if i < 100 else 0
def one_dim_discrete():
    """A single agent along the x-axis."""

    VERSION = 'scrimmage-v0'
    _write_temp_mission(x_discrete=True, ctrl_y=False, y_discrete=True,
                        num_actors=1, end=1000)
    combine_actors = False
    global_sensor = False
    env, obs, total_reward = \
        _run_test(VERSION, combine_actors, global_sensor, _get_action)

    assert len(obs[0]) == 1
    assert obs[0][0] == 0
    assert isinstance(env.action_space, gym.spaces.Discrete)
    assert isinstance(env.observation_space, gym.spaces.Box)
    assert env.action_space.n == 2
    assert total_reward == 4
