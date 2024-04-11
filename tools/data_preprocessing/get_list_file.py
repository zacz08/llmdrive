import os
import sys

from tqdm import tqdm

if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("Usage: python get_list_file.py $DATASET_ROOT")
        sys.exit(0)
    else:
        dataset_root = sys.argv[1]
    list_file = os.path.join(dataset_root, 'dataset_index.txt')
    path = os.path.join(dataset_root, 'data')
    towns = os.listdir(path)
    with open(list_file, 'w') as f:
        for town in towns:
            routes = os.listdir(os.path.join(path, town))
            for route in tqdm(routes):
                if os.path.isdir(os.path.join(path, town, route)):
                    frames = len(os.listdir(os.path.join(path, town, route, 'measurements')))
                    if frames < 32:
                        print("Route %s only havs %d frames (<32). We have omitted it!" % (route, frames))
                    else:
                        f.write(route + ' ' + str(frames) + '\n')
