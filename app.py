import can
import time
import threading
from flask import Flask, jsonify

app = Flask(__name__)

data = {
    "vehicle_speed": 0.0,
    "engine_speed": None,
    "wheel_based_speed": 0.0,
    "fuel_level": 0.0,
    "trip_distance": 0.0,
    "total_distance": 0.0,
    "engine_load": None,
    "engine_temp": None,
    "vin": "",
    "def_level": 0.0,
    "engine_hours": 0.0,
    "status": "OFF"
}

last_msg_time = 0.0
was_off = True

trip_start_total_distance = None

fuel_tank_1 = None
fuel_tank_2 = None

vin_buffer = bytearray()
vin_expected_size = 0
vin_done = False

last_engine_hours_request_time = 0.0
ENGINE_HOURS_REQUEST_INTERVAL = 5.0


def extract_pgn(arbitration_id: int) -> int:
    pf = (arbitration_id >> 16) & 0xFF
    ps = (arbitration_id >> 8) & 0xFF

    if pf < 240:
        return pf << 8
    return (pf << 8) | ps


def kmh_to_mph(kmh: float) -> float:
    return kmh * 0.621371


def reset_runtime() -> None:
    global fuel_tank_1, fuel_tank_2, trip_start_total_distance

    data["vehicle_speed"] = 0.0
    data["engine_speed"] = None
    data["wheel_based_speed"] = 0.0
    data["engine_load"] = None
    data["engine_temp"] = None
    data["fuel_level"] = 0.0
    data["trip_distance"] = 0.0
    data["status"] = "OFF"

    fuel_tank_1 = None
    fuel_tank_2 = None
    trip_start_total_distance = None


def decode_rpm(d: bytes):
    if len(d) < 5:
        return None
    raw = d[3] | (d[4] << 8)
    return None if raw >= 0xFFFA else raw * 0.125


def decode_engine_load(d: bytes):
    if len(d) < 3:
        return None
    return None if d[2] == 0xFF else float(d[2])


def decode_speed(d: bytes):
    if len(d) < 3:
        return None
    raw = d[1] | (d[2] << 8)
    return None if raw == 0xFFFF else raw * 0.00390625  # km/h


def decode_temp(d: bytes):
    if len(d) < 1:
        return None
    return None if d[0] == 0xFF else d[0] - 40


def decode_fuel(d: bytes):
    if len(d) < 2:
        return None
    return None if d[1] == 0xFF else d[1] * 0.4


def decode_distance(d: bytes):
    if len(d) < 4:
        return None
    raw = d[0] | (d[1] << 8) | (d[2] << 16) | (d[3] << 24)
    return None if raw == 0xFFFFFFFF else raw * 0.125


def decode_engine_hours(d: bytes):
    if len(d) < 4:
        return None
    raw = d[0] | (d[1] << 8) | (d[2] << 16) | (d[3] << 24)
    return None if raw == 0xFFFFFFFF else raw * 0.05


def decode_def(d: bytes):
    if len(d) < 1:
        return None
    return None if d[0] == 0xFF else d[0] * 0.4


def request_pgn(bus, requested_pgn: int, label: str) -> None:
    pgn_bytes = [
        requested_pgn & 0xFF,
        (requested_pgn >> 8) & 0xFF,
        (requested_pgn >> 16) & 0xFF,
    ]

    msg = can.Message(
        arbitration_id=0x18EAFF00,  # Request to global address
        data=pgn_bytes + [0xFF] * 5,
        is_extended_id=True
    )
    bus.send(msg)
    print(f"{label} requested")


def request_vin(bus) -> None:
    request_pgn(bus, 0x00FEEC, "VIN")


def request_engine_hours(bus) -> None:
    request_pgn(bus, 0x00FEE5, "Engine Hours")


def can_reader() -> None:
    global last_msg_time, was_off, trip_start_total_distance
    global fuel_tank_1, fuel_tank_2
    global vin_buffer, vin_expected_size, vin_done
    global last_engine_hours_request_time

    bus = can.interface.Bus(channel="can0", interface="socketcan")

    request_vin(bus)
    request_engine_hours(bus)
    last_engine_hours_request_time = time.time()

    print("CAN thread started")

    while True:
        msg = bus.recv(timeout=1)
        now = time.time()

        if now - last_engine_hours_request_time >= ENGINE_HOURS_REQUEST_INTERVAL:
            try:
                request_engine_hours(bus)
            except Exception as e:
                print(f"Engine Hours request error: {e}")
            last_engine_hours_request_time = now

        if msg:
            last_msg_time = now
        elif now - last_msg_time > 3:
            if data["status"] != "OFF":
                reset_runtime()
                was_off = True
            continue

        if not msg or not msg.is_extended_id:
            continue

        if was_off:
            data["status"] = "ON"
            was_off = False
            trip_start_total_distance = None
        else:
            data["status"] = "ON"

        pgn = extract_pgn(msg.arbitration_id)

        if pgn == 61444:  # EEC1
            data["engine_speed"] = decode_rpm(msg.data)
            data["engine_load"] = decode_engine_load(msg.data)

        elif pgn == 65265:  # CCVS
            speed = decode_speed(msg.data)
            if speed is not None:
                mph = kmh_to_mph(speed)
                data["vehicle_speed"] = mph
                data["wheel_based_speed"] = mph

        elif pgn == 65262:  # ET1
            temp = decode_temp(msg.data)
            if temp is not None:
                data["engine_temp"] = temp

        elif pgn == 65276:  # fuel tank 1
            f = decode_fuel(msg.data)
            if f is not None:
                fuel_tank_1 = f

        elif pgn == 65277:  # fuel tank 2
            f = decode_fuel(msg.data)
            if f is not None:
                fuel_tank_2 = f

        elif pgn == 65248:  # distance
            dist = decode_distance(msg.data)
            if dist is not None:
                data["total_distance"] = dist

                if trip_start_total_distance is None:
                    trip_start_total_distance = dist

                if dist >= trip_start_total_distance:
                    data["trip_distance"] = dist - trip_start_total_distance

        elif pgn == 65253:  # engine hours
            h = decode_engine_hours(msg.data)
            if h is not None:
                data["engine_hours"] = h

        elif pgn == 65110:  # DEF (may be proprietary / simulator specific)
            d = decode_def(msg.data)
            if d is not None:
                data["def_level"] = d

        elif pgn == 0xEC00:  # TP.CM
            if len(msg.data) >= 3 and msg.data[0] == 32:  # BAM
                vin_expected_size = msg.data[1] | (msg.data[2] << 8)
                vin_buffer = bytearray()
                vin_done = False

        elif pgn == 0xEB00:  # TP.DT
            if not vin_done:
                vin_buffer.extend(msg.data[1:])
                if len(vin_buffer) >= vin_expected_size:
                    try:
                        data["vin"] = vin_buffer[:vin_expected_size].decode(
                            errors="ignore"
                        ).strip()
                    except Exception:
                        pass
                    vin_done = True

        if fuel_tank_1 is not None and fuel_tank_2 is not None:
            data["fuel_level"] = (fuel_tank_1 + fuel_tank_2) / 2
        elif fuel_tank_1 is not None:
            data["fuel_level"] = fuel_tank_1


@app.route("/telemetry", methods=["GET"])
def get_data():
    return jsonify(data)


if __name__ == "__main__":
    t = threading.Thread(target=can_reader, daemon=True)
    t.start()

    app.run(port=8080)