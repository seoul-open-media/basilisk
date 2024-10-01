import tkinter as tk


class StageMonitor(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        tk.Label(self, text="Stage Monitor").pack()
        tk.Canvas(self, background="lightblue").pack()