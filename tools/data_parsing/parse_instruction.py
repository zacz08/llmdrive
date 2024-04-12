import os
import re
import sys
import json
from multiprocessing import Pool

from tqdm import tqdm
from matplotlib import pyplot as plt

from turn_rules import Turn01, Turn02, Turn03, Turn04, Turn05, Turn06
from follow_rules import Follow01, Follow02, Follow03, Follow04
from notice_rules import Notice01, Notice02, Notice03, Notice04, Notice05, Notice06, Notice07, Notice08
from other_rules import Other01, Other02, Other03, Other04, Other05

registered_class = {
    'Turn-01-L': Turn01(direction='left'),
    'Turn-01-R': Turn01(direction='right'),
    'Turn-01-L-dis': Turn01(direction='left', dis=True),
    'Turn-01-R-dis': Turn01(direction='right', dis=True),
    'Turn-02-L': Turn02(direction='left'),
    'Turn-02-R': Turn02(direction='right'),
    'Turn-02-S': Turn02(direction='straight'),
    'Turn-02-L-dis': Turn02(direction='left', dis=True),
    'Turn-02-R-dis': Turn02(direction='right', dis=True),
    'Turn-02-S-dis': Turn02(direction='straight', dis=True),
    'Turn-03-L': Turn03(direction='left'),
    'Turn-03-R': Turn03(direction='right'),
    'Turn-03-S': Turn03(direction='straight'),
    'Turn-03-L-dis': Turn03(direction='left', dis=True),
    'Turn-03-R-dis': Turn03(direction='right', dis=True),
    'Turn-03-S-dis': Turn03(direction='straight', dis=True),
    'Turn-04-L': Turn04(direction='left'),
    'Turn-04-R': Turn04(direction='right'),
    'Turn-04-S': Turn04(direction='straight'),
    'Turn-04-L-dis': Turn04(direction='left', dis=True),
    'Turn-04-R-dis': Turn04(direction='right', dis=True),
    'Turn-04-S-dis': Turn04(direction='straight', dis=True),
    'Turn-05-1': Turn05(exit_no=1),
    'Turn-05-2': Turn05(exit_no=2),
    'Turn-05-3': Turn05(exit_no=3),
    'Turn-06-L-L': Turn06(first_direction='left', second_direction='left'),
    'Turn-06-L-R': Turn06(first_direction='left', second_direction='right'),
    'Turn-06-L-S': Turn06(first_direction='left', second_direction='straight'),
    'Turn-06-R-L': Turn06(first_direction='right', second_direction='left'),
    'Turn-06-R-R': Turn06(first_direction='right', second_direction='right'),
    'Turn-06-R-S': Turn06(first_direction='right', second_direction='straight'),
    'Turn-06-S-L': Turn06(first_direction='straight', second_direction='left'),
    'Turn-06-S-R': Turn06(first_direction='straight', second_direction='right'),
    'Turn-06-S-S': Turn06(first_direction='straight', second_direction='straight'),

    'Follow-01-L': Follow01(direction='left'),
    'Follow-01-R': Follow01(direction='right'),
    'Follow-01-L-dis': Follow01(direction='left', dis=True),
    'Follow-01-R-dis': Follow01(direction='right', dis=True),
    'Follow-02-s1': Follow02(style=1),
    'Follow-02-s2': Follow02(style=2),
    'Follow-02-s1-dis': Follow02(style=1, dis=True),
    'Follow-02-s2-dis': Follow02(style=2, dis=True),
    'Follow-03-s1': Follow03(style=1),
    'Follow-03-s2': Follow03(style=2),
    'Follow-03-s1-dis': Follow03(style=1, dis=True),
    'Follow-03-s2-dis': Follow03(style=2, dis=True),
    'Follow-04-L': Follow04(directions=['left','straight']),
    'Follow-04-R': Follow04(directions=['right']),
    'Follow-04-L-dis': Follow04(directions=['left','straight'], dis=True),
    'Follow-04-R-dis': Follow04(directions=['right'], dis=True),

    'Notice-01': Notice01(),
    'Notice-02': Notice02(),
    'Notice-03': Notice03(),
    'Notice-04': Notice04(),
    'Notice-05': Notice05(),
    'Notice-06': Notice06(),
    'Notice-07': Notice07(),
    'Notice-08-R': Notice08(light_status='red'),
    'Notice-08-G': Notice08(light_status='green'),
    'Notice-08-Y': Notice08(light_status='yellow'),

    'Other-01': Other01(),
    'Other-02': Other02(),
    'Other-03': Other03(),
    'Other-04': Other04(),
    'Other-05': Other05(),
}

rule_id_mapping_dict = {}
for i, key in enumerate(registered_class.keys()):
    rule_id_mapping_dict[key] = i

processing_rules = ['Turn-03-L','Turn-03-R','Turn-03-S','Turn-03-L-dis','Turn-03-R-dis','Turn-03-S-dis','Turn-01-L','Turn-01-R','Turn-01-L-dis','Turn-01-R-dis','Follow-01-L','Follow-01-R','Follow-01-L-dis','Follow-01-R-dis','Follow-02-s1','Follow-02-s1-dis','Follow-02-s2','Follow-02-s2-dis','Turn-06-L-L','Turn-06-L-R','Turn-06-L-S','Turn-06-R-L','Turn-06-R-R','Turn-06-R-S','Turn-06-S-L','Turn-06-S-R','Turn-06-S-S','Turn-02-L','Turn-02-R','Turn-02-S','Turn-02-L-dis','Turn-02-R-dis','Turn-02-S-dis','Turn-04-L','Turn-04-R','Turn-04-S','Turn-04-L-dis','Turn-04-R-dis','Turn-04-S-dis','Follow-03-s1','Follow-03-s2','Follow-03-s1-dis','Follow-03-s2-dis','Follow-04-L','Follow-04-R','Follow-04-L-dis','Follow-04-R-dis','Turn-05-1','Turn-05-2','Turn-05-3','Other-01','Other-02','Other-03','Other-04','Other-05']

def process(line):
    try:
        processed_data = []
        path, frames = line.split()
        # dir_path = os.path.join(dataset_root, , path)
        frames = int(frames.strip())
        # town_id = int(re.findall(r'town(\d\d)', dir_path)[0])
        # weather_id = int(re.findall(r'_w(\d+)_', dir_path)[0])
        town_id = int(re.findall(r'town(\d\d)', path)[0])
        weather_id = int(re.findall(r'_w(\d+)_', path)[0])
        json_data = []
        for frame_id in tqdm(range(frames)):
            # full_path = os.path.join(dir_path, "Town%02d/", 'measurements_full', "%04d.json" % town_id % frame_id)
            full_path = os.path.join(dataset_root, 'data', f"Town{town_id:02d}", path, 'measurements_full', f"{frame_id:04d}.json")
            value_json = json.load(open(full_path, 'r'))
            json_data.append(value_json)

        for rule in processing_rules:
            results = registered_class[rule].process({'data': json_data, 'town_id': town_id, 'weather_id': weather_id})
            rule_id = rule_id_mapping_dict[rule]
            for result in results:
                result['instruction'] = rule
                result['instruction_id'] = rule_id
                result['town_id'] = town_id
                result['weather_id'] = weather_id
                result['route_path'] = path
                result['route_frames'] = int(line.split()[1])
                result['bad_case'] = 'False'
                processed_data.append(result)

        return processed_data

    except Exception as e:
        except_type, except_value, except_traceback = sys.exc_info()
        except_file = os.path.split(except_traceback.tb_frame.f_code.co_filename)[1]
        exc_dict = {
            "Error type: ": except_type,
            "Error information": except_value,
            "Error file": except_file,
            "Error line": except_traceback.tb_lineno,
        }
        print(exc_dict)
        print(town_id)
        print(rule)
        print(full_path)

if __name__ == '__main__':
    dataset_root = sys.argv[1]
    list_file = os.path.join(dataset_root, 'dataset_index.txt')
    lines = open(list_file, 'r').readlines()

    with Pool(8) as p:
        r_list = list(tqdm(p.imap(process, lines), total=len(lines)))

    f_write = open(os.path.join(dataset_root, 'navigation_instruction_list.txt'), 'w')
    for results in r_list:
        if not results:
            continue
        for result in results:
            processed_json = json.dumps(result)
            f_write.write(processed_json+'\n')
    f_write.close()
