import os
import json
import tkinter as tk
import tkinter.messagebox as messagebox
import tkinter.filedialog as filedialog
from seq import Sequence


class SequenceEditor(tk.Frame):
    def __init__(self, parent, filepath, closer):
        super().__init__(parent)
        self.filepath = filepath
        self.closer = closer
        self.pack()

        filename_frame = tk.Frame(self)
        filename_frame.pack()
        filename_keylabel = tk.Label(filename_frame, text="File:")
        filename_keylabel.pack(side="left")
        self.filename_vallabel = tk.Label(
            filename_frame, text=os.path.basename(filepath))
        self.filename_vallabel.pack(side="left")

        buttons_frame = tk.Frame(self)
        buttons_frame.pack()
        self.save_btn = tk.Button(
            buttons_frame, text="Save", command=lambda: self.save(save_as=False))
        self.save_btn.pack(side="left")
        self.save_as_btn = tk.Button(
            buttons_frame, text="Save As", command=lambda: self.save(save_as=True))
        self.save_as_btn.pack(side="left")
        self.play_btn = tk.Button(
            buttons_frame, text="Play", command=self.play)
        self.play_btn.pack(side="left")
        self.stop_btn = tk.Button(
            buttons_frame, text="Stop", command=self.stop)
        self.stop_btn.pack(side="left")
        self.close_btn = tk.Button(
            buttons_frame, text="Close", command=self.close)
        self.close_btn.pack(side="left")

        self.load()

    def close(self):
        self.closer.open_filenames.remove(os.path.basename(self.filepath))
        self.destroy()

    def load(self):
        self.seq = Sequence(self.filepath)
        # self.canvas = tk.Canvas(self, background="pink")
        # self.canvas.pack()

    def save(self, save_as):
        pass
    #     if save_as:
    #         new_filepath = tk.filedialog.asksaveasfilename(
    #             defaultextension=".json",
    #             filetypes=[
    #                 ("JSON files",
    #                  "*.json"), ("All files", "*.*")
    #             ],
    #             initialdir=self.master.path
    #         )
    #         if not new_filepath:
    #             return
    #         save_to = new_filepath
    #     else:
    #         save_to = self.filepath

    #     try:
    #         for key, editor in self.editors.items():
    #             self.data[key] = editor.get_val()

    #         with open(save_to, 'w') as file:
    #             json.dump(self.data, file, indent=4)

    #         tk.messagebox.showinfo("Success", "File saved successfully.")

    #         if save_as:
    #             self.filepath = save_to
    #             self.filename_vlbl.config(text=os.path.basename(self.filepath))
    #             self.master.load()
    #     except Exception as e:
    #         tk.messagebox.showerror("Error", f"Could not save file: {e}")

    def play(self):
        self.seq.start()

    def stop(self):
        self.seq.stop()
