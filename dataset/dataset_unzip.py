"""
    @author: Cheng Zhang
    
    BRIEF:  Run this script in the path of LMDrive dataset to unzip the original zipped files
    
"""

import os
# for exit control
import sys

import zipfile
import tarfile
from pathlib import Path


def unzip_file(file_path, target_path, type='tar.gz', remove_pkg=False):
    # open the zip file
    # zFile = zipfile.ZipFile(file_zip_path+'/frames.zip', "r")

    if type == 'tar.gz':
        if not '.tar.gz' in file_path:
            print('File ', file_path, ' is not a tar.gz package, Skipped...')
        else:
            with tarfile.open(file_path, 'r:gz') as tar:
                tar.extractall(path=target_path)
                print(f"{file_path} extracted finish.")
        
        if remove_pkg:
            os.remove(file_path)

    elif type == 'zip':
        if not 'zip' in file_path:
            print('File ', file_path, ' is not a .zip package, Skipped...')
        else:
            zFile = zipfile.ZipFile(file_path, "r")
            Path(zFile.extract(file_path))
            zFile.close()
            print('Unzipped file', file_path + '/frames.zip')
        


if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("Usage: python dataset_unzip.py $DATASET_ROOT")
        sys.exit(0)
    else:
        dataset_root = sys.argv[1]

    path = os.path.join(dataset_root, 'data')
    towns = os.listdir(path)
    for town in towns:
        target_path = os.path.join(path, town)
        package_list = os.listdir(target_path)
        for i in package_list:
            targz_file_path = os.path.join(target_path, i)
            unzip_file(targz_file_path, target_path, type='tar.gz')
                
    print("#### Unzip finished successfully! ####")
    sys.exit(0)

    