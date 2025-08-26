from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query
from fastapi.responses import JSONResponse
import asyncio
import time
import psutil
import subprocess
import logging
import platform
from threading import Lock

from camera_controller import (
    init_all_cameras_from_config,
    capture_image,
    get_cams,
    get_lock
)
from plc_controller import init_plc, connect_plc, monitor_trigger, write_signal

app = FastAPI()

clients = set()
logging.basicConfig(level=logging.INFO)

async def send_system_status():
    """Gửi thông tin CPU/RAM/DISK và số lượng camera cho tất cả client WS."""
    while True:
        cpu = psutil.cpu_percent()
        disk = psutil.disk_usage('/').percent
        ram = psutil.virtual_memory().percent
        cams = get_cams()
        cam_count = len(cams)

        for ws in list(clients):
            try:
                await ws.send_json({
                    "type": "system_status",
                    "cpu": cpu,
                    "storage": disk,
                    "ram": ram,
                    "system_ok": True,
                    "active": cam_count,
                })
            except:
                clients.discard(ws)
        await asyncio.sleep(5)

async def handle_capture_and_send(websocket: WebSocket):
    """Chụp ảnh từ tất cả camera và gửi về client WS."""
    processing_times = []
    cams = get_cams()

    with get_lock():
        if not cams:
            await websocket.send_json({"type": "error", "message": "No camera"})
            return

        for cam_info in cams:
            try:
                start_time = time.time()
                image_data = capture_image(cam_info)
                processing_time_ms = (time.time() - start_time) * 1000
                processing_times.append(processing_time_ms)

                if image_data:
                    await websocket.send_json({
                        "type": "camera_image",
                        "camera": cam_info["name"],
                        "image": image_data,
                        "processing_time_ms": processing_time_ms
                    })
            except Exception as e:
                print(f"⚠️ Lỗi capture {cam_info['name']}: {e}")

    avg_time = sum(processing_times) / len(processing_times) if processing_times else 0
    await websocket.send_json({
        "type": "processing_summary",
        "total_processing": avg_time,
        "total_cameras": len(cams)
    })

async def ping_clients_loop():
    """Ping WS client định kỳ để kiểm tra kết nối."""
    while True:
        await asyncio.sleep(10)
        for ws in list(clients):
            try:
                await ws.send_json({"ping": "ping"})
            except:
                clients.discard(ws)

async def run_plc_trigger():
    print("⚡ PLC Trigger → Capturing images...")
    cams = get_cams()
    if clients and cams:
        for ws in list(clients):
            try:
                await handle_capture_and_send(ws)
            except:
                clients.discard(ws)
    else:
        print("⚠️ No WS clients or no cameras connected, skip capture")

@app.on_event("startup")
def startup_event():
    """Chạy khi server khởi động."""
    # Kết nối PLC
    if init_plc():
        try:
            connect_plc()
            print("✅ PLC connected")
        except Exception as e:
            print(f"❌ PLC connect failed: {e}")

    # Init camera
    with get_lock():
        init_all_cameras_from_config()

    # Tạo các task chạy nền
    asyncio.create_task(ping_clients_loop())
    asyncio.create_task(send_system_status())
    asyncio.create_task(monitor_trigger(run_plc_trigger, interval=0.1))

@app.websocket("/ws/image")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket"""
    await websocket.accept()
    clients.add(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            if data == "capture":
                await handle_capture_and_send(websocket)
            else:
                await websocket.send_json({"type": "info", "message": f"Server nhận: {data}"})
    except WebSocketDisconnect:
        clients.discard(websocket)
    except Exception as e:
        clients.discard(websocket)
        print(f"⚠️ WS error: {e}")

@app.get("/capture")
async def capture_all():
    cams = get_cams()
    if not cams:
        return {"status": "error", "message": "No cameras available"}
    if not clients:
        return {"status": "error", "message": "No WS clients connected"}

    for ws in list(clients):
        try:
            await handle_capture_and_send(ws)
        except:
            clients.discard(ws)

    return {"status": "ok", "message": "Images sent to WS clients"}

@app.get("/reload_camera")
def reload_camera():
    """Reload danh sách camera từ CameraConfig.json."""
    with get_lock():
        init_all_cameras_from_config()
    return {"status": "ok", "message": "Cameras reloaded"}

@app.get("/writetoplc")
async def test_write_signal(
    name: str = Query(..., description="Tên biến PLC, ví dụ: result hoặc Q0.1"),
    value: int = Query(..., description="Giá trị cần ghi: 0 hoặc 1")
):
    """Ghi giá trị tín hiệu vào PLC."""
    try:
        bool_value = bool(value)
        await write_signal(name, bool_value)
        return {"status": "ok", "signal": name, "value": bool_value}
    except Exception as e:
        return JSONResponse(status_code=500, content={
            "status": "exception",
            "error": str(e)
        })

@app.get("/scancamera")
def scan_camera():
    """Gọi script check_and_save_cameras.py để scan camera."""
    try:
        process = subprocess.run(
            ["python", "check_and_save_cameras.py"],
            capture_output=True,
            text=True
        )
        if process.returncode == 0:
            return {"status": "success", "output": process.stdout.strip()}
        else:
            return JSONResponse(status_code=500, content={
                "status": "error",
                "output": process.stderr.strip()
            })
    except Exception as e:
        return JSONResponse(status_code=500, content={
            "status": "exception",
            "output": str(e)
        })

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("ws_server:app", host="0.0.0.0", port=8000, reload=True)
