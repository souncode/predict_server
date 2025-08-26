import os
import cv2
import json
import base64
import time
from datetime import datetime
from ctypes import *
import numpy as np
from threading import Lock
from MvCameraControl_class import *

PixelType_Gvsp_Mono8        = 0x01080001
PixelType_Gvsp_BayerRG8     = 0x01080009
PixelType_Gvsp_RGB8_Packed  = 0x02180014

CAPTURE_DIR = "captures"
os.makedirs(CAPTURE_DIR, exist_ok=True)

cams = []
cams_lock = Lock()
camera_configs = []
issaveimage = False
save_path = "images"

def load_config(path="CameraConfig.json"):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def load_camera_config():
    global camera_configs, issaveimage, save_path
    with open("CameraConfig.json", "r") as f:
        data = json.load(f)
        camera_configs = data["cameras"]
        issaveimage = data.get("issave", False)
        save_path = data.get("save_path", "images")

def find_device_by_serial_or_ip(dev_list, target_serial=None, target_ip=None):
    for i in range(dev_list.nDeviceNum):
        dev_info = cast(dev_list.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        serial, ip = None, None

        if dev_info.nTLayerType == MV_GIGE_DEVICE:
            raw = bytes(dev_info.SpecialInfo.stGigEInfo.chSerialNumber)
            serial = raw.decode("utf-8", errors="ignore").strip("\x00")
            nip1 = (dev_info.SpecialInfo.stGigEInfo.nCurrentIp >> 24) & 0xFF
            nip2 = (dev_info.SpecialInfo.stGigEInfo.nCurrentIp >> 16) & 0xFF
            nip3 = (dev_info.SpecialInfo.stGigEInfo.nCurrentIp >> 8) & 0xFF
            nip4 = dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xFF
            ip = f"{nip1}.{nip2}.{nip3}.{nip4}"
        elif dev_info.nTLayerType == MV_USB_DEVICE:
            raw = bytes(dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber)
            serial = raw.decode("utf-8", errors="ignore").strip("\x00")

        if (target_serial and serial == target_serial) or (target_ip and ip == target_ip):
            return dev_info
    return None

def init_all_cameras_from_config(config_path="CameraConfig.json"):
    global cams
    cams.clear()

    config = load_config(config_path)
    dev_list = MV_CC_DEVICE_INFO_LIST()
    ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, dev_list)
    if ret != 0 or dev_list.nDeviceNum == 0:
        print("❌ Không tìm thấy camera nào")
        return []

    for cam_cfg in config["cameras"]:
        dev_info = find_device_by_serial_or_ip(
            dev_list,
            target_serial=cam_cfg.get("serial"),
            target_ip=cam_cfg.get("ip")
        )
        if dev_info is None:
            print(f"❌ Không tìm thấy {cam_cfg['name']}")
            continue

        cam = MvCamera()
        if cam.MV_CC_CreateHandle(dev_info) != 0:
            continue
        if cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0) != 0:
            cam.MV_CC_DestroyHandle()
            continue

        if dev_info.nTLayerType == MV_GIGE_DEVICE:
            pkt = cam.MV_CC_GetOptimalPacketSize()
            if int(pkt) > 0:
                cam.MV_CC_SetIntValue("GevSCPSPacketSize", pkt)

        cam.MV_CC_SetEnumValue("PixelFormat", PixelType_Gvsp_BayerRG8)
        cam.MV_CC_SetEnumValue("TriggerMode", 1)
        cam.MV_CC_SetEnumValue("TriggerSource", 7)
        cam.MV_CC_SetFloatValue("ExposureTime", float(cam_cfg.get("exposure_time", 5000)))
        cam.MV_CC_SetFloatValue("Gain", float(cam_cfg.get("gain", 10.0)))
        cam.MV_CC_StartGrabbing()

        cams.append({
            "cam": cam,
            "name": cam_cfg["name"],
            "cfg": cam_cfg,
            "dev_info": dev_info
        })

    return cams

def save_bgr(bgr, save_path, prefix, issave=True):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"{prefix}_{ts}.jpg"
    filepath = os.path.join(save_path, filename)
    if issave:
        cv2.imwrite(filepath, bgr)
    return filepath

def decode_frame(buf, frame_info, payload):
    w, h, pxtype = frame_info.nWidth, frame_info.nHeight, frame_info.enPixelType
    size_used = frame_info.nFrameLen
    if size_used <= 0 or size_used > payload:
        size_used = w * h * (3 if pxtype == PixelType_Gvsp_RGB8_Packed else 1)
    np_flat = np.frombuffer(buf, dtype=np.uint8, count=size_used)

    if pxtype == PixelType_Gvsp_RGB8_Packed:
        return np_flat.reshape((h, w, 3))
    elif pxtype == PixelType_Gvsp_Mono8:
        gray = np_flat.reshape((h, w))
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    else:
        bayer = np_flat.reshape((h, w))
        return cv2.cvtColor(bayer, cv2.COLOR_BAYER_RG2BGR)

def capture_image(cam_info, issave=True, save_path=CAPTURE_DIR):
    try:
        cam = cam_info["cam"]
        name = cam_info["name"]

        intval = MVCC_INTVALUE()
        memset(byref(intval), 0, sizeof(intval))
        cam.MV_CC_GetIntValue("PayloadSize", intval)
        payload = int(intval.nCurValue)

        buf = (c_ubyte * payload)()
        frame_info = MV_FRAME_OUT_INFO_EX()

        cam.MV_CC_SetCommandValue("TriggerSoftware")
        ret = cam.MV_CC_GetOneFrameTimeout(byref(buf), payload, frame_info, 1000)
        if ret != 0:
            print(f"❌ {name}: GetOneFrameTimeout fail (ret={ret})")
            return None

        bgr = decode_frame(buf, frame_info, payload)
        save_bgr(bgr, save_path, prefix=name, issave=issave)
        _, buffer = cv2.imencode(".jpg", bgr)
        return f"data:image/jpeg;base64,{base64.b64encode(buffer).decode('utf-8')}"

    except Exception as e:
        print(f"⚠️ Lỗi capture {cam_info['name']}: {e}")
        return None

def get_cams():
    return cams

def get_lock():
    return cams_lock
