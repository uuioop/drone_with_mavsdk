# Copyright (c) 2020 Ultralytics
# This file is part of ultralytics, released under the MIT license.
# https://opensource.org/licenses/MIT

import os
import re
import glob
import time
import logging
import subprocess
from pathlib import Path

import requests
import torch


def gsutil_getsize(url=''):
    # gsutil du -sh https://storage.googleapis.com/ultralytics/yolov5
    s = ['gsutil du -sh', url]
    return subprocess.check_output(s, shell=True).decode('utf-8').split('\t')[0]


def attempt_download(file, repo='ultralytics/yolov5'):  # from utils.google_utils import *; attempt_download()
    # Attempt file download from GitHub release if not found locally
    file = Path(str(file).strip().replace("'", ''))
    if not file.exists():
        try:
            response = requests.get(f'https://api.github.com/repos/{repo}/releases/latest').json()  # github api
            assets = [x['name'] for x in response['assets']]  # release assets, i.e. ['yolov5s.pt', 'yolov5m.pt', ...]
            tag = response['tag_name']  # i.e. 'v1.0'
        except:
            assets = ['yolov5s.pt', 'yolov5m.pt', 'yolov5l.pt', 'yolov5x.pt', 'yolov5n.pt', 'yolov5n6.pt']
            tag = 'v6.1'  # current version
        name = file.name
        if name in assets:
            msg = f'{file} missing, try downloading from https://github.com/{repo}/releases/'
            redundant = False  # second download
            try:
                torch.hub.download_url_to_file(f'https://github.com/{repo}/releases/download/{tag}/{name}',
                                              file,
                                              progress=redundant)
            except Exception as e:
                print(f'Download error: {e}')
                try:
                    from google_drive_downloader import google_drive_downloader  # pip install googledrivedownloader
                    print(f'Attempting to download {name} from Google Drive...')
                    google_drive_downloader.download_file_from_google_drive(file_id, str(file))
                except:
                    print(f'All download attempts failed')
                    return
            print(f'Successfully downloaded {name}')
        else:
            print(f'WARNING: file {name} not found in {repo} release assets {assets}')
    return str(file) 