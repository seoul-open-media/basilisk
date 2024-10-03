import serial
import serial.tools.list_ports
import time
import struct
import threading
from queue import Queue
import json


mode_to_num = {
    None: 0,
    "Idle": 0,
    "Free": 3,
    "DoPreset": 4,
    "SetMags": 5,
    "SetPhis": 7,
    "Pivot": 9,
    "PivSpin": 13,
    "RandomMags": 19,
    "WalkToDir": 21,
    "WalkToPos": 22,
    "Sufi": 23,
    "Orbit": 24,
    "Diamond": 25,
    "RandomWalk": 26,
    "GhostWalk": 27,
}
oneshot_to_num = {
    None: 0,
    "CRMuxXbee": 1,
    "SetBaseYaw": 2,
    "Inspire": 3
}

keys_uint8 = [
    "strengths",
    "didimbal",
    "steps",
]
keys_uint16 = [
    "idx",
    "duration",
    "min_dur",
    "max_dur",
    "min_phase_dur",
    "max_phase_dur",
]
keys_float32 = [
    "offset",
    "tgt_phi_L",
    "tgt_phi_R",
    "tgt_phispeed_L",
    "tgt_phispeed_R",
    "tgt_yaw",
    "stride",
    "bend_L",
    "bend_R",
    "speed",
]


def preview_key_order():
    print(*sorted(keys_uint8+keys_uint16+keys_float32), sep="\n")


class XbeeCommandSender:
    def __init__(self):
        port = self.find_port()
        if port:
            try:
                self.port = serial.Serial(port, 115200)
                print(f"Serial port {port} initialized")
            except serial.SerialException as e:
                print(f"Error: Failed initializing serial port: {e}")
        else:
            print("Serial port will not be used")

        self.cmd_queue = Queue()

        self.running = True
        self.thread = threading.Thread(target=self._send)
        self.thread.daemon = True  # Ensures thread exits when the main program does
        self.thread.start()

        print("Key order preview:")
        print("---BEGIN---")
        preview_key_order()
        print("---END---")

    def find_port(self):
        for port in serial.tools.list_ports.comports():
            if port.description.startswith("FT231X USB UART"):
                print(f"Xbee found on port {port.device}")
                return port.device
        print("WARNING: Xbee NOT found")
        return None

    def numerize(self, val):
        if isinstance(val, str):
            if val.lower() == "nan":
                return float('nan')
            try:
                int_val = int(val)
                return int_val
            except ValueError:
                try:
                    float_val = float(val)
                    return float_val
                except ValueError:
                    raise ValueError(f"Cannot numerize value: {val}")
        return val

    def queue_cmd(self, cmd, suids):  # cmd = Command path or Preset index
        if isinstance(cmd, str):
            with open(cmd, 'r') as f:
                data = json.load(f)
        elif isinstance(cmd, int):
            data = {}
            data["what"] = "Mode_DoPreset"
            data["idx"] = cmd
        else:
            return

        if not "what" in data:
            return

        if data["what"].startswith("Mode_"):
            mode = data["what"][5:]
            oneshot = None
        elif data["what"].startswith("Oneshot_"):
            mode = None
            oneshot = data["what"][8:]

        suid_bits = 0
        if 0 in suids:
            suid_bits = (1 << 13) - 1
        else:
            for suid in suids:
                suid_bits |= (1 << (suid - 1))

        cmd_bytes = bytearray(50)
        cmd_bytes[:8] = [255, 255, 255, 255, suid_bits & 0xFF, suid_bits >> 8,
                         oneshot_to_num[oneshot], mode_to_num[mode]]

        byte_index = 8
        for key in sorted(data.keys()):  # Just sort alphabetically
            if key != "what":
                val = self.numerize(data[key])

                if key in keys_uint8:
                    packed_value = struct.pack('<B', val)
                elif key in keys_uint16:
                    packed_value = struct.pack('<H', val)
                elif key in keys_float32:
                    packed_value = struct.pack('<f', val)
                else:
                    continue

                if byte_index + len(packed_value) > 50:
                    raise ValueError(
                        f"Command bytes exceeds 50 for SUID {suid}")

                cmd_bytes[byte_index:byte_index +
                          len(packed_value)] = packed_value
                byte_index += len(packed_value)

        self.cmd_queue.put(cmd_bytes)

        print("Xbee Command queued:")
        print(list(cmd_bytes))

    def _send(self):
        while True:
            if not self.cmd_queue.empty():
                cmd_bytes = self.cmd_queue.get()
                self.port.write(cmd_bytes)
                print("Xbee Command sent:")
                print(list(cmd_bytes))
            time.sleep(0.001)


xb_cs = XbeeCommandSender()
