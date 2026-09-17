import os
import subprocess

DEPTH_IDS = [("8086","0b07"),("8086","0b3a"),("2bc5","0403"),("2bc5","0502"),("2bc5","0511")]
DEPTH_NAMES = {"8086:0b07":"Intel RealSense D435","8086:0b3a":"Intel RealSense D415",
               "2bc5:0403":"Orbbec Astra","2bc5:0502":"Orbbec Astra Pro","2bc5:0511":"Orbbec 3D"}
LIDAR_IDS = [("10c4","ea60"),("1a86","7523")]
LIDAR_NAMES = {"10c4:ea60":"RPLidar C1","1a86:7523":"RPLidar A1"}

def _has_usb_id(id_list):
    try:
        out = subprocess.check_output(["lsusb"], text=True)
        for vid, pid in id_list:
            if f"{vid}:{pid}" in out: return True
    except: pass
    return False

def has_depth_camera():
    return _has_usb_id(DEPTH_IDS)

def has_lidar():
    if _has_usb_id(LIDAR_IDS): return True
    for dev in ['/dev/mower_lidar','/dev/ttyUSB0','/dev/ttyUSB1','/dev/lidar']:
        if os.path.exists(dev): return True
    return False

def has_usb_camera():
    if has_depth_camera(): return False
    for i in range(4):
        if os.path.exists(f"/dev/video{i}"): return True
    return False

def get_device_names():
    """返回检测到的设备型号列表"""
    devices = []
    try:
        out = subprocess.check_output(["lsusb"], text=True)
        for vid, pid in DEPTH_IDS:
            key = f"{vid}:{pid}"
            if key in out:
                devices.append(DEPTH_NAMES.get(key, f"深度相机({key})"))
                break
        for vid, pid in LIDAR_IDS:
            key = f"{vid}:{pid}"
            if key in out:
                devices.append(LIDAR_NAMES.get(key, f"激光雷达({key})"))
                break
    except:
        pass
    return devices or ["未识别设备"]

def auto_detect():
    depth = has_depth_camera()
    lidar = has_lidar()
    cam = has_usb_camera()
    if depth and lidar: return 1
    elif lidar: return 2
    elif cam: return 0
    else: return -1
