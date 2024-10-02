from json_list import JsonList
from seq_editor import SequenceEditor


class SequenceList(JsonList):
    def __init__(self, parent):
        super().__init__(parent, "seqs", "Sequence")

    def get_editor(self, filepath):
        return SequenceEditor(self.master.seqeditorlist, filepath, self)
