import os
import json

def emptyJson(file_path: str) -> None:
    with open(file_path, 'w') as file:
        json.dump({}, file)

def writeJson(file_path: str, data) -> None:
    with open(file_path, 'w') as json_file:
        json.dump(data, json_file, indent=4)

def openJson(file_path: str):
    with open(file_path, 'r') as json_file:
        data = json.load(json_file)
    return data

def checkExisting(path: str) -> bool:
    return os.path.exists(path)