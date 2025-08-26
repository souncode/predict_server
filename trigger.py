# trigger_sync.py
import cv2
import numpy as np
from ctypes import *
from MvCameraControl_class import *
from datetime import datetime
import time
import os
import json
import threading

PixelType_Gvsp_Mono8        = 0x01080001
PixelType_Gvsp_BayerRG8     = 0x01080009
PixelType_Gvsp_RGB8_Packed  = 0x02180014


def save_bgr(img_bgr, save_dir, prefix="capture", issave=False):
    if issave:
        os.makedirs(save_dir, exist_ok=True)
        fname = os.path.join(
            save_dir,
            f"{prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.jpg"
        )
        cv2.imwrite(fname, img_bgr)
        print(f"💾 {prefix} -> {fname}")
    else:
        cv2.imshow(prefix, img_bgr)
        cv2.waitKey(200)
        cv2.destroyAllWindows()


def load_config(path="CameraConfig.json"):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


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

        print(f"🔎 Camera {i+1}: serial={serial}, ip={ip}")
        if (target_serial and serial == target_serial) or (target_ip and ip == target_ip):
            return dev_info
    return None


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


def camera_worker(dev_info, cam_cfg, issave, save_path, trigger_event):
    cam = MvCamera()
    if cam.MV_CC_CreateHandle(dev_info) != 0:
        print(f"❌ {cam_cfg['name']}: CreateHandle fail")
        return
    if cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0) != 0:
        print(f"❌ {cam_cfg['name']}: OpenDevice fail")
        cam.MV_CC_DestroyHandle()
        return
    print(f"✅ Đã mở {cam_cfg['name']}")

    if dev_info.nTLayerType == MV_GIGE_DEVICE:
        pkt = cam.MV_CC_GetOptimalPacketSize()
        if int(pkt) > 0:
            cam.MV_CC_SetIntValue("GevSCPSPacketSize", pkt)

    cam.MV_CC_SetEnumValue("PixelFormat", PixelType_Gvsp_BayerRG8)
    cam.MV_CC_SetEnumValue("TriggerMode", 1)
    cam.MV_CC_SetEnumValue("TriggerSource", 7)
    cam.MV_CC_SetFloatValue("ExposureTime", float(cam_cfg.get("exposure_time", 5000)))
    cam.MV_CC_SetFloatValue("Gain", float(cam_cfg.get("gain", 10.0)))

    intval = MVCC_INTVALUE()
    memset(byref(intval), 0, sizeof(intval))
    cam.MV_CC_GetIntValue("PayloadSize", intval)
    payload = int(intval.nCurValue)

    cam.MV_CC_StartGrabbing()
    buf = (c_ubyte * payload)()
    frame_info = MV_FRAME_OUT_INFO_EX()

    # Chờ signal trigger
    trigger_event.wait()

    # Sau khi có signal -> chụp
    cam.MV_CC_SetCommandValue("TriggerSoftware")
    ret = cam.MV_CC_GetOneFrameTimeout(byref(buf), payload, frame_info, 1000)
    if ret != 0:
        print(f"❌ {cam_cfg['name']}: GetOneFrameTimeout fail (ret={ret})")
    else:
        bgr = decode_frame(buf, frame_info, payload)
        save_bgr(bgr, save_path, prefix=cam_cfg['name'], issave=issave)

    cam.MV_CC_StopGrabbing()
    cam.MV_CC_CloseDevice()
    cam.MV_CC_DestroyHandle()
    print(f"✅ Hoàn tất {cam_cfg['name']}")


def main():
    config = load_config("CameraConfig.json")
    issave = config.get("issave", False)
    save_path = config.get("save_path", "./captures")

    dev_list = MV_CC_DEVICE_INFO_LIST()
    ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, dev_list)
    if ret != 0 or dev_list.nDeviceNum == 0:
        print("❌ Không tìm thấy camera")
        return
    print(f"🔍 Phát hiện {dev_list.nDeviceNum} camera")

    trigger_event = threading.Event()
    threads = []
    for cam_cfg in config["cameras"]:
        dev_info = find_device_by_serial_or_ip(
            dev_list,
            target_serial=cam_cfg.get("serial"),
            target_ip=cam_cfg.get("ip")
        )
        if dev_info is None:
            print(f"❌ Không tìm thấy {cam_cfg['name']}")
            continue
        t = threading.Thread(
            target=camera_worker,
            args=(dev_info, cam_cfg, issave, save_path, trigger_event),
            daemon=True
        )
        threads.append(t)
        t.start()

    time.sleep(1)  # cho tất cả thread ready
    print("🚦 Gửi lệnh TRIGGER đồng bộ cho tất cả camera")
    trigger_event.set()  # đồng loạt chụp

    for t in threads:
        t.join()
    print("🎉 Tất cả camera đã chụp xong")


if __name__ == "__main__":
    main()
