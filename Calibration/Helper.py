import os
import time

def nothing(x):
    pass

def create_directory(dir_name):
    try:
        print('making dir ... ', end='\t')
        os.mkdir(dir_name)
    except:
        print('already exist')
    print('done')