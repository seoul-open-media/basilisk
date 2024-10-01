import os
import json
import tkinter as tk
import tkinter.messagebox as messagebox
import tkinter.filedialog as filedialog
from .field_editor import MinimalEditor
from xbee_cs import xb_cs


class CommandEditor(tk.Toplevel):
    def __init__(self, parent, filepath):
        super().__init__(parent)
        self.title("Command Editor")
        self.filepath = filepath
        self.editors = {}
        self.protocol("WM_DELETE_WINDOW", self.close)

        filename_frm = tk.Frame(self)
        filename_frm.pack()
        filename_klbl = tk.Label(filename_frm, text="File:")
        filename_klbl.pack(side="left")
        self.filename_vlbl = tk.Label(
            filename_frm, text=os.path.basename(filepath))
        self.filename_vlbl.pack(side="left")

        buttons_frm = tk.Frame(self)
        buttons_frm.pack()
        self.save_btn = tk.Button(
            buttons_frm, text="Save", command=lambda: self.save(save_as=False))
        self.save_btn.pack(side="left")
        self.save_as_btn = tk.Button(
            buttons_frm, text="Save As", command=lambda: self.save(save_as=True))
        self.save_as_btn.pack(side="left")
        self.send_btn = tk.Button(buttons_frm, text="Send", command=self.send)
        self.send_btn.pack(side="left")
        self.close_btn = tk.Button(
            buttons_frm, text="Close", command=self.close)
        self.close_btn.pack(side="left")

        self.editors_frm = tk.Frame(self)
        self.editors_frm.pack()

        self.load()

    def load(self):
        row = 0
        try:
            with open(self.filepath, 'r') as file:
                self.data = json.load(file)
                for key, val in self.data.items():
                    key_lbl = tk.Label(self.editors_frm, text=key)
                    key_lbl.grid(row=row, column=0)
                    editor = MinimalEditor(self.editors_frm, val)
                    self.editors[key] = editor
                    editor.grid(row=row, column=1)
                    row += 1
        except Exception as e:
            tk.messagebox.showerror("Error", f"Could not load file: {e}")

    def save(self, save_as):
        if save_as:
            new_filepath = tk.filedialog.asksaveasfilename(
                defaultextension=".json",
                filetypes=[
                    ("JSON files",
                     "*.json"), ("All files", "*.*")
                ],
                initialdir=self.master.path
            )
            if not new_filepath:
                return
            save_to = new_filepath
        else:
            save_to = self.filepath

        try:
            for key, editor in self.editors.items():
                self.data[key] = editor.get_val()

            with open(save_to, 'w') as file:
                json.dump(self.data, file, indent=4)

            tk.messagebox.showinfo("Success", "File saved successfully.")
        except Exception as e:
            tk.messagebox.showerror("Error", f"Could not save file: {e}")

        if save_as:
            self.master.load()
            self.filepath = save_to
            self.filename_vlbl.config(text=os.path.basename(self.filepath))

    def send(self):
        xb_cs.send(self.filepath, [0])

    def close(self):
        self.master.open_filenames.remove(os.path.basename(self.filepath))
        self.destroy()
