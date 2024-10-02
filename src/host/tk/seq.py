import os
import time
import threading
import json
from xbee_cs import xb_cs


class Sequence:
    def __init__(self, filepath):
        self.data = {}
        self.filepath = filepath
        self.load()
        self.playing = False
        self.thread = None
        self.child_seq = None

    def load(self):
        if not os.path.isfile(self.filepath):
            raise FileNotFoundError(f"{self.filepath} file not found")
        with open(self.filepath, 'r') as file:
            self.data = json.load(file)
            self.data["seq"].sort(key=lambda x: x["time"])

    def start(self):
        if not self.data.get("seq"):
            print("No sequence to play")
            return

        if self.playing:
            self.stop()
        print(f"Starting sequence: {self.filepath}")

        self.playing = True
        self.thread = threading.Thread(target=self._play)
        self.thread.start()

    def _play(self):
        start_time = time.time()
        cur_node_idx = 0
        nodes = self.data["seq"]
        timlen = self.data["len"]
        while self.playing:
            if time.time() - start_time > timlen:
                self.playing = False
                return

            if cur_node_idx < len(nodes):
                node = nodes[cur_node_idx]
                node_start_time = node["time"]
                current_time = time.time() - start_time

                if current_time >= node_start_time:
                    self.emit(node)
                    cur_node_idx += 1

            time.sleep(0.01)
        print(f"Sequence complete: {self.filepath}")

    def stop(self):
        print(f"Stopping sequence: {self.filepath}")
        self.playing = False
        if self.child_seq is not None:
            self.child_seq.stop()
            self.child_seq = None
        if self.thread.is_alive():
            self.thread.join()

    def emit(self, node):
        node_type = node["type"]
        if node_type == "Preset":
            xb_cs.queue_cmd(node["idx"], node["suids"])
        elif node_type == "Command_Blip":
            cmd_path = os.path.join(
                os.path.dirname(__file__),
                "cmds",
                node["name"] + ".json"
            )
            xb_cs.queue_cmd(cmd_path, node["suids"])
        elif node_type == "Command_AndWait":
            pass
        elif node_type == "Sequence":
            loop = 1
            try:
                loop = node["loop"]
            except:
                pass
            for i in range(loop):
                pass

    ######################################################
    # See you later

    def save(self):
        pass

    def add(self, t, node, suids, loop):
        if "seq" not in self.data:
            self.data["seq"] = []

        if isinstance(suids, list):
            self.data["seq"].append({"time": t, "node": node, "suids": suids})
        elif isinstance(loop, int):
            self.data["seq"].append({"time": t, "node": node, "loop": loop})

        self.data["seq"].sort(key=lambda x: x["time"])

    def delete(self, node):
        pass
