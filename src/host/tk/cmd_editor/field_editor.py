import tkinter as tk


class FieldEditor(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)

    def get_val(self):
        raise NotImplementedError


class MinimalEditor(FieldEditor):
    def __init__(self, parent, val):
        super().__init__(parent)

        self.val_entry = tk.Entry(self)
        self.val_entry.insert(0, val)
        self.val_entry.pack(side="left")

    def get_val(self):
        return self.val_entry.get()
