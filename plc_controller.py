import json
import snap7
from snap7.util import get_bool, set_bool
from snap7.types import Areas
import asyncio
import os
import re
from threading import Lock

# --- Global ---
plc_client = None
plc_config = None
signals = None
plc_lock = Lock()  # đảm bảo thread-safe

AREA_MAP = {
    "M": Areas.MK,
    "Q": Areas.PA,
    "I": Areas.PE
}


def load_plc_config(path="PlcConfig.json"):
    if not os.path.exists(path):
        print(f"❌ Config file {path} not found")
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except json.JSONDecodeError:
        print("❌ Invalid JSON format in PLC config")
        return None


def init_plc(path="PlcConfig.json"):
    global plc_config, signals
    cfg = load_plc_config(path)
    if cfg:
        plc_config = cfg["plc"]
        signals = cfg["signals"]
        return True
    return False


def connect_plc():
    global plc_client
    if plc_config is None:
        raise Exception("❌ PLC config not initialized. Call init_plc() first.")
    with plc_lock:
        if plc_client is None:
            plc_client = snap7.client.Client()
        if not plc_client.get_connected():
            plc_client.connect(plc_config["ip"], plc_config["rack"], plc_config["slot"])
            if not plc_client.get_connected():
                raise Exception("❌ PLC connection failed")
    print(f"✅ Connected to PLC at {plc_config['ip']}")


def safe_read_signal(name):
    with plc_lock:
        if not plc_client or not plc_client.get_connected():
            connect_plc()
        sig = signals[name]
        area = AREA_MAP[sig["area"]]
        data = plc_client.read_area(area, 0, sig["byte"], 1)
        return get_bool(data, 0, sig["bit"])


def safe_write_signal(name: str, value: bool):
    with plc_lock:
        if not plc_client or not plc_client.get_connected():
            connect_plc()

        if signals and name in signals:
            sig = signals[name]
            area = AREA_MAP[sig["area"]]
            byte = sig["byte"]
            bit = sig["bit"]
        else:
            match = re.match(r'^(M|Q|I)(\d+)\.(\d+)$', name)
            if match:
                area = AREA_MAP[match.group(1)]
                byte = int(match.group(2))
                bit = int(match.group(3))
            else:
                raise ValueError(f"❌ Invalid signal name or address: {name}")

        # Đọc byte hiện tại
        data = bytearray(plc_client.read_area(area, 0, byte, 1))
        set_bool(data, 0, bit, value)
        plc_client.write_area(area, 0, byte, data)
        print(f"✅ Wrote {value} to {name} (area={area}, byte={byte}, bit={bit})")


async def read_signal(name):
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, safe_read_signal, name)


async def write_signal(name, value):
    loop = asyncio.get_running_loop()
    await loop.run_in_executor(None, safe_write_signal, name, value)


async def monitor_trigger(callback, interval=0.05):
    """
    Monitor PLC signal 'trigger' for rising edge.
    Calls `callback()` when triggered.
    """
    print("🔍 PLC trigger monitor started...")
    last_state = False
    while True:
        try:
            current_state = await read_signal("trigger")
            if current_state and not last_state:
                print("🚨 Trigger detected! Executing callback...")
                # Gửi tín hiệu done ON -> OFF
                await write_signal("done", True)
                await asyncio.sleep(0.02)  # nhỏ để tránh lỗi
                await write_signal("done", False)
                await callback()
            last_state = current_state
        except Exception as e:
            print(f"❌ PLC read error: {e}")
        await asyncio.sleep(interval)

