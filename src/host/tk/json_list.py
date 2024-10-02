import os
import tkinter as tk
import tkinter.messagebox as messagebox


class JsonList(tk.Frame):
    def __init__(self, parent, dirname, label):
        super().__init__(parent)
        self.path = os.path.join(os.path.dirname(__file__), dirname)
        self.open_filenames = []

        title_frame = tk.Frame(self)
        title_frame.pack(fill="x")
        title_label = tk.Label(title_frame, text=label + " List")
        title_label.pack(fill="x")

        path_frame = tk.Frame(self)
        path_frame.pack(fill="x")
        path_keylabel = tk.Label(path_frame, text="Path:")
        path_keylabel.pack(side="left")
        path_vallabel = tk.Label(path_frame, text=self.path)
        path_vallabel.pack(side="left", fill="x")

        btns_frame = tk.Frame(self)
        btns_frame.pack(fill="x")
        reload_btn = tk.Button(btns_frame, text="Reload", command=self.load)
        reload_btn.pack(side="left", fill="x", expand=True)
        create_btn = tk.Button(btns_frame, text="Create",
                               command=self.create)
        create_btn.pack(side="left", fill="x", expand=True)
        delete_btn = tk.Button(btns_frame, text="Delete", command=self.delete)
        delete_btn.pack(side="left", fill="x", expand=True)

        list_frame = tk.Frame(self)
        list_frame.pack(fill='both', expand=True)
        self.list = tk.Listbox(list_frame)
        self.list.pack(side="left", fill='both', expand=True)
        self.list.bind("<Double-Button-1>", self.open_editor)
        list_scrollbar = tk.Scrollbar(
            list_frame, orient="vertical", command=self.list.yview)
        self.list.configure(yscrollcommand=list_scrollbar.set)
        list_scrollbar.pack(side="right", fill="y")

        self.load()

    def load(self):
        self.list.delete(0, tk.END)
        try:
            json_files = sorted([f for f in os.listdir(
                self.path) if f.endswith(".json")])
            for file in json_files:
                self.list.insert(tk.END, file)
        except Exception as e:
            tk.messagebox.showerror("Error", f"Could NOT load JSON files: {e}")

    def create(self):
        tk.messagebox.showerror(
            "Sorry", f"Open Template or Blank file then use Save As")

    def delete(self):
        selection = self.list.curselection()
        if selection:
            filename = self.list.get(selection[0])
            filepath = os.path.join(self.path, filename)
            if tk.messagebox.askyesno("Confirm Delete",  f"You sure you want to delete '{filename}'?"):
                try:
                    os.remove(filepath)
                    self.load()
                except Exception as e:
                    tk.messagebox.showerror(
                        "Error", f"Could not delete file: {e}")

    def get_editor(self, filepath):
        raise NotImplementedError

    def open_editor(self, event):
        selection = self.list.curselection()
        if selection:
            filename = self.list.get(selection[0])
            if filename not in self.open_filenames:
                filepath = os.path.join(self.path, filename)
                self.get_editor(filepath)
                self.open_filenames.append(filename)
