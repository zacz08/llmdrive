import os

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
