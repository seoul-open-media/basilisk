from json_list import JsonList
from cmd_editor.cmd_editor import CommandEditor


class CommandList(JsonList):
    def __init__(self, parent):
        super().__init__(parent, "cmds", "Command")

    def get_editor(self, filepath):
        return CommandEditor(self, filepath)
