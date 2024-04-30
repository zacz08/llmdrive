"""
    @author: Cheng Zhang
    
    BRIEF:  Run this script in the path of LMDrive dataset to unzip the original zipped files
    
"""

import os
# for exit control
import sys
import logging
import zipfile
import tarfile
from pathlib import Path
_logger = logging.getLogger("train")


def unzip_file(file_path, target_path, type='tar.gz', remove_pkg=False):
    # open the zip file
    # zFile = zipfile.ZipFile(file_zip_path+'/frames.zip', "r")

    if type == 'tar.gz':
        if not '.tar.gz' in file_path:
            _logger.warning(f"{file_path} extracted finish.")
            # print('File ', file_path, ' is not a tar.gz package, Skipped...')
        else:
            with tarfile.open(file_path, 'r:gz') as tar:
                tar.extractall(path=target_path)
                tar.close()
            _logger.info(f"{file_path} extracted finish.")
            # print(f"{file_path} extracted finish.")
        
        if remove_pkg:
            os.remove(file_path)

    elif type == 'zip':
        if not 'zip' in file_path:
            print('File ', file_path, ' is not a .zip package, Skipped...')
        else:
            zFile = zipfile.ZipFile(file_path, "r")
            Path(zFile.extract(file_path))
            zFile.close()
            _logger.info('Unzipped file', file_path + '/frames.zip')
            # print('Unzipped file', file_path + '/frames.zip')
        


if __name__ == "__main__":
    if len(sys.argv) != 3:
        _logger.error("Usage: python dataset_unzip.py $DATASET_ROOT $UNZIP_SAVE_PATH")
        # print("Usage: python dataset_unzip.py $DATASET_ROOT $UNZIP_SAVE_PATH")
        sys.exit(0)
    else:
        package_download_root = sys.argv[1]
        save_path_root = sys.argv[2]

    towns = os.listdir(os.path.join(package_download_root, 'data'))
    for town in towns:
        package_list = os.listdir(os.path.join(package_download_root, 'data', town))
        target_path = os.path.join(save_path_root, 'data', town)
        for i in package_list:
            targz_file_path = os.path.join(package_download_root, 'data', town, i)
            unzip_file(targz_file_path, target_path, type='tar.gz', remove_pkg=True)
                
    # print("#### Unzip finished successfully! ####")
    _logger.info("#### Unzip finished successfully! ####")
    sys.exit(0)

    