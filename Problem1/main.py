import json

if __name__ == "__main__":
    with open("best_strategies.json") as json_file:
        best_strategies = json.load(json_file)

    with open("current_strategy.json") as json_file:
        curr_strat = json.load(json_file)

    env_name = curr_strat['env_name']
    mdl_name = curr_strat['mdl_name']
    delta_q = curr_strat['delta_q']
    k = curr_strat['k']
    rrt_strat = curr_strat['rrt_strategy']

    env_name = curr_strat['env_name']
    mdl_name = curr_strat['mdl_name']
    delta_q = curr_strat['delta_q']
    k = curr_strat['k']
    rrt_strat = curr_strat['rrt_strategy']

    with open(str(env_name) + ".json") as json_file:
        desc = json.load(json_file)

    bounding_poly = desc['bounding_polygon']
    obstacles = [desc[key] for key, val in desc.items() if key.startswith('obs')]
    pos_start = desc['pos_start']
    pos_goal = desc['pos_goal']
    vel_start = desc['vel_start']
    vel_goal = desc['vel_goal']
    dt = desc['vehicle_dt']
    v_max = desc['vehicle_v_max']
    a_max = desc['vehicle_a_max']
    omega_max = desc['vehicle_omega_max']
    phi_max = desc['vehicle_phi_max']
    vehicle_length = desc['vehicle_L']