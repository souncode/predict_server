import snap7
from snap7.types import Areas
import time

PLC_IP = '192.168.1.82'
RACK = 0
SLOT = 1

# Mapping area từ ký tự sang snap7 constant
AREA_MAP = {
    "I": Areas.PE,  # Input
    "Q": Areas.PA,  # Output
    "M": Areas.MK   # Merker
}

client = snap7.client.Client()
client.connect(PLC_IP, RACK, SLOT)

def set_bit(area: str, byte: int, bit: int, value: bool):
    """Set or reset a specific bit in I/Q/M area"""
    area = area.upper()
    if area not in AREA_MAP:
        raise ValueError("Area phải là I, Q hoặc M")

    snap_area = AREA_MAP[area]

    # Đọc 1 byte (8 bit)
    data = client.read_area(snap_area, 0, byte, 1)
    
    if value:
        data[0] |= (1 << bit)   # Bật bit
    else:
        data[0] &= ~(1 << bit)  # Tắt bit

    # Ghi lại vào PLC
    client.write_area(snap_area, 0, byte, data)
    print(f"✅ Set {area}{byte}.{bit} = {value}")

if client.get_connected():
    print(f"✅ Connected to PLC at {PLC_IP}")

    # Ví dụ bật M0.2
    set_bit("M", 0, 2, True)
    time.sleep(2)
    # Tắt M0.2
    set_bit("M", 0, 2, False)

    # Bật Q0.2
    set_bit("Q", 0, 2, True)
    time.sleep(2)
    set_bit("Q", 0, 2, False)

    client.disconnect()
else:
    print("❌ Không kết nối được PLC")
