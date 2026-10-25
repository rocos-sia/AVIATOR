"""Native Tk styling; checkbox artwork needs no image assets or extra libraries."""
import math

BG = "#edf2f6"
CARD = "#ffffff"
INK = "#172d40"
MUTED = "#687d8e"
ACCENT = "#007f79"


def configure(root):
    import tkinter as tk
    from tkinter import font, ttk

    family = "Noto Sans CJK SC" if "Noto Sans CJK SC" in font.families(root) else "sans"
    root.configure(background=BG)
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure(".", font=(family, 10), background=BG, foreground=INK)
    style.configure("TFrame", background=BG)
    style.configure("Card.TFrame", background=CARD)
    style.configure("TLabel", background=BG, foreground=INK)
    style.configure("Brand.TLabel", font=(family, 25, "bold"))
    style.configure("Muted.TLabel", foreground=MUTED, font=(family, 9))
    style.configure("Card.TLabel", background=CARD)
    style.configure("Heading.TLabel", background=CARD, font=(family, 12, "bold"))
    style.configure("Note.TLabel", background=CARD, foreground=MUTED, font=(family, 9))
    style.configure("Status.TLabel", background=CARD, font=(family, 11, "bold"))
    style.configure("Badge.TLabel", background="#e1efeb", foreground="#246a59", padding=(12, 5), font=(family, 9))
    style.configure("TSeparator", background="#e5edf1")
    style.configure("TButton", background=CARD, foreground=INK, bordercolor="#cedae2",
                    lightcolor=CARD, darkcolor=CARD, borderwidth=1, padding=(14, 9),
                    focuscolor=ACCENT, focusthickness=1)
    style.map("TButton", background=[("disabled", "#f2f5f7"), ("pressed", "#dce9ec"), ("active", "#edf5f5")],
              foreground=[("disabled", "#9aaab6")])
    style.configure("Primary.TButton", background=ACCENT, foreground="white",
                    bordercolor=ACCENT, lightcolor=ACCENT, darkcolor=ACCENT, font=(family, 10, "bold"))
    style.map("Primary.TButton", background=[("disabled", "#d5e4e4"), ("pressed", "#005f5c"), ("active", "#006f69")],
              foreground=[("disabled", "#7c9696"), ("!disabled", "white")],
              bordercolor=[("disabled", "#d5e4e4"), ("!disabled", ACCENT)])
    style.configure("Stop.TButton", foreground="#ad5048")
    style.map("Stop.TButton", foreground=[("disabled", "#9aaab6"), ("!disabled", "#ad5048")])
    style.configure("TEntry", fieldbackground="#f6f9fb", bordercolor="#d5dfe6", padding=(9, 8),
                    lightcolor="#f6f9fb", darkcolor="#f6f9fb", insertcolor=INK)
    style.map("TEntry", bordercolor=[("focus", ACCENT)])
    style.configure("Vertical.TScrollbar", background="#cedae2", troughcolor="#f1f5f8",
                    borderwidth=0, arrowsize=12)
    style.layout("Log.Vertical.TScrollbar", [("Vertical.Scrollbar.trough", {"sticky": "ns", "children": [
        ("Vertical.Scrollbar.thumb", {"expand": "1", "sticky": "nswe"})]})])
    style.configure("Log.Vertical.TScrollbar", background="#375364", troughcolor="#142735",
                    bordercolor="#142735", lightcolor="#375364", darkcolor="#375364",
                    borderwidth=0, width=11)
    style.map("Log.Vertical.TScrollbar", background=[("active", "#54788e")])

    # A single tick, drawn at the display scale, replaces clam's default X mark.
    size = max(20, round(21 * float(root.tk.call("tk", "scaling")) / (4 / 3)))
    def rgb(value):
        return tuple(int(value[i:i+2], 16) for i in (1, 3, 5))

    def segment_distance(x, y, a, b):
        vx, vy = b[0] - a[0], b[1] - a[1]
        t = max(0, min(1, ((x-a[0])*vx + (y-a[1])*vy)/(vx*vx+vy*vy)))
        return math.hypot(x-a[0]-t*vx, y-a[1]-t*vy)

    def indicator(selected=False, disabled=False, hover=False):
        edge = "#d7e1e7" if disabled else ACCENT if selected or hover else "#afc0cb"
        fill = "#e4ecef" if disabled else ACCENT if selected else CARD
        tick = "#92a8b4" if disabled else CARD
        def pixel(x, y):
            # Rounded box in a 21-unit coordinate system, with room to its right.
            qx, qy = abs(x-9.5)-5.5, abs(y-10.5)-5.5
            distance = math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0) - 3
            color = CARD if distance > 0 else edge if distance > -1.3 else fill
            if selected and min(segment_distance(x, y, (5.3, 10.5), (8.3, 13.5)),
                                segment_distance(x, y, (8.3, 13.5), (13.8, 7.5))) < 1:
                color = tick
            return rgb(color)
        image = tk.PhotoImage(master=root, width=size+7, height=size)
        rows = []
        for y in range(size):
            row = []
            for x in range(size+7):
                samples = [pixel((x+(sx+.5)/3)*21/size, (y+(sy+.5)/3)*21/size)
                           for sx in range(3) for sy in range(3)]
                row.append("#" + "".join(f"{round(sum(p[c] for p in samples)/9):02x}" for c in range(3)))
            rows.append("{" + " ".join(row) + "}")
        image.put(" ".join(rows))
        return image

    images = [indicator(), indicator(True), indicator(disabled=True),
              indicator(True, True), indicator(hover=True)]
    root._checkbox_images = images  # Tk does not keep Python image objects alive.
    style.element_create("Aviator.indicator", "image", images[0],
                         ("disabled", "selected", images[3]), ("disabled", images[2]),
                         ("selected", images[1]), ("active", images[4]), sticky="w")
    style.layout("Aviator.TCheckbutton", [("Checkbutton.padding", {"sticky": "nswe", "children": [
        ("Aviator.indicator", {"side": "left", "sticky": "w"}),
        ("Checkbutton.focus", {"side": "left", "sticky": "w", "children": [
            ("Checkbutton.label", {"sticky": "nswe"})]})]})])
    style.configure("Aviator.TCheckbutton", background=CARD, foreground=INK,
                    font=(family, 10, "bold"), padding=(0, 4), focuscolor=ACCENT)
    style.map("Aviator.TCheckbutton", background=[("active", CARD)],
              foreground=[("disabled", "#a1b0bb"), ("!disabled", INK)])
    return family
