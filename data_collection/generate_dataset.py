##### number of server #####
server_num = 1

##### routes setting #####
routes = {}
# routes["training_routes/routes_town01_short.xml"] = "scenarios/town01_all_scenarios.json"
routes["training_routes/routes_town01_tiny.xml"] = "scenarios/town01_all_scenarios.json"
# routes["training_routes/routes_town02_short.xml"] = "scenarios/town02_all_scenarios.json"
routes["training_routes/routes_town02_tiny.xml"] = "scenarios/town02_all_scenarios.json"
# routes["training_routes/routes_town03_short.xml"] = "scenarios/town03_all_scenarios.json"
# routes["training_routes/routes_town03_tiny.xml"] = "scenarios/town03_all_scenarios.json"
# routes["training_routes/routes_town04_short.xml"] = "scenarios/town04_all_scenarios.json"
# routes["training_routes/routes_town04_tiny.xml"] = "scenarios/town04_all_scenarios.json"
# routes["training_routes/routes_town05_short.xml"] = "scenarios/town05_all_scenarios.json"
# routes["training_routes/routes_town05_tiny.xml"] = "scenarios/town05_all_scenarios.json"
# routes["training_routes/routes_town05_long.xml"] = "scenarios/town05_all_scenarios.json"
# routes["training_routes/routes_town06_short.xml"] = "scenarios/town06_all_scenarios.json"
# routes["training_routes/routes_town06_tiny.xml"] = "scenarios/town06_all_scenarios.json"
# routes["training_routes/routes_town07_short.xml"] = "scenarios/town07_all_scenarios.json"
# routes["training_routes/routes_town07_tiny.xml"] = "scenarios/town07_all_scenarios.json"
# routes["training_routes/routes_town10_short.xml"] = "scenarios/town10_all_scenarios.json"
# routes["training_routes/routes_town10_tiny.xml"] = "scenarios/town10_all_scenarios.json"
# routes["additional_routes/routes_town01_long.xml"] = "scenarios/town01_all_scenarios.json"
# routes["additional_routes/routes_town02_long.xml"] = "scenarios/town02_all_scenarios.json"
# routes["additional_routes/routes_town03_long.xml"] = "scenarios/town03_all_scenarios.json"
# routes["additional_routes/routes_town04_long.xml"] = "scenarios/town04_all_scenarios.json"
# routes["additional_routes/routes_town06_long.xml"] = "scenarios/town06_all_scenarios.json"


import os
from bash_tools import generate_batch_collect, generate_bashs



def main():
    # init dictionary
    if server_num < 1 or server_num > 4:
        print("Server number must be an integer between 1 and 4.")
    else:
        for i in range(server_num):
            if not os.path.exists("sub-%d" % i):
                os.mkdir("sub-%d" % i)
            if not os.path.exists("sub-%d/results" % i):
                os.mkdir("sub-%d/results" % i)

    generate_bashs(routes, server_num)
    generate_batch_collect(routes, server_num)



if __name__=='__main__':
    main()