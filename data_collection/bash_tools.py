import os
import random

ip_ports = []

for port in range(2000, 2008, 2):
    ip_ports.append(("localhost", port, port + 500))


carla_seed = 2000
# traffic_seed = 2000


def generate_script(
    ip, port, tm_port, route, scenario, carla_seed, traffic_seed, save_path
):
    lines = []
    lines.append("export HOST=%s\n" % ip)
    lines.append("export PORT=%d\n" % port)
    lines.append("export TM_PORT=%d\n" % tm_port)
    lines.append("export ROUTES=${LEADERBOARD_ROOT}/data/%s\n" % route)
    lines.append("export SCENARIOS=${LEADERBOARD_ROOT}/data/%s\n" % scenario)
    lines.append("export CARLA_SEED=%d\n" % carla_seed)
    lines.append("export TRAFFIC_SEED=%d\n" % traffic_seed)
    lines.append("export TEAM_CONFIG=${YAML_ROOT}/auto_agent.yaml\n")
    lines.append("export SAVE_PATH=${DATA_ROOT}/%s/data\n" % save_path)
    lines.append(
        "export CHECKPOINT_ENDPOINT=${YAML_ROOT}/%s/results/%s.json\n"
        % (save_path, route.split("/")[1].split(".")[0])
    )
    lines.append("\n")
    base = open("base_script.sh").readlines()

    for line in lines:
        base.insert(13, line)

    return base

    
def generate_batch_collect(routes, server_num):
    routes_list = []
    for route in routes:
        routes_list.append(route.split("/")[1].split(".")[0])

    if not os.path.exists("batch_run"):
        os.mkdir("batch_run")

    for route in routes_list:
        fw = open("batch_run/run_route_%s.sh" % route, "w")
        for i in range(server_num):
            fw.write("bash data_collection/bashs/sub-%d/%s.sh & \n" % (i, route))


def generate_bashs(routes, server_num):
    for i in range(server_num):
        if not os.path.exists("bashs"):
            os.mkdir("bashs")
        if not os.path.exists("bashs/sub-%d" % i):
            os.mkdir("bashs/sub-%d" % i)
        for route in routes:
            ip, port, tm_port = ip_ports[i]
            traffic_seed = random.randint(1000, 8000)
            carla_seed = random.randint(1000, 8000)
            script = generate_script(
                ip,
                port,
                tm_port,
                route,
                routes[route],
                carla_seed,
                traffic_seed,
                'sub-%d' % i
            )
            fw = open(
                "bashs/sub-%d/%s.sh" % (i, route.split("/")[1].split(".")[0]), "w"
            )
            for line in script:
                fw.write(line)
