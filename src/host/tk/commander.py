import tkinter as tk
from cmd_list import CommandList
from seq_list import SequenceList
from stage_monitor import StageMonitor


class BasiliskCommander(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("BasilskCommander")

        self.stage_monitor_frm = StageMonitor(self)
        self.stage_monitor_frm.pack(side=tk.TOP, fill=tk.X, expand=False)

        self.cmdlist = CommandList(self)
        self.cmdlist.pack(side=tk.LEFT, fill=tk.Y, expand=False)

        self.seqeditorlist = tk.Frame(self)
        self.seqeditorlist.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        seqeditorlist_title_frame = tk.Frame(self.seqeditorlist)
        seqeditorlist_title_frame.pack(fill="x")
        seqeditorlist_title_label = tk.Label(
            seqeditorlist_title_frame, text="Sequence Editor List")
        seqeditorlist_title_label.pack(fill="x")

        self.seqlist = SequenceList(self)
        self.seqlist.pack(side=tk.RIGHT, fill=tk.Y, expand=False)
