# json.py
# Created: Friday, 11th March 2022 3:06:37 pm
# Matthew Riche
# Last Modified: Friday, 11th March 2022 3:06:41 pm
# Modified By: Matthew Riche

import json
import os
import pymel.core as pm


def get_path():
    '''
    Get the local path "dynamically" admid the in-progress python interpretation.
    '''

    mod_path = os.path.abspath(__file__)
    parent_path = os.path.dirname(mod_path)

    return parent_path


def dump_to_file(dict, path):
    '''
    Given a single dict, dump it to a file.
    
    Will fail if there's non-serialized content, like PyNode, PyMel attribute classes, etc.
    '''

    try:
        with open(path, 'w') as json_file:
            json.dump(dict, json_file)
        return True

    except:
        print("FAILED TO WRITE {}".format(path))
        return False


def read_from_file(path):
    '''
    Read dictionary contents from JSON
    '''

    try:
        file = open(path)
        data = json.load(file)

        file.close()

        return data
    except:
        print("FAILED TO READ {}".format(path))
        return None