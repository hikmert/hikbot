#!/usr/bin/env python3
"""Map manager GUI: auto-save every 60 s + save-as / load / clear buttons."""

import glob
import os
import time
import threading
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, messagebox

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from slam_toolbox.srv import DeserializePoseGraph, Reset, SerializePoseGraph

MAP_DIR = os.path.expanduser('~/otonom_projeler/hikbot/maps')
MAP_FILE = os.path.join(MAP_DIR, 'warehouse_map')
AUTO_SAVE_INTERVAL = 60


def _strip_posegraph(path: str) -> str:
    """slam_toolbox appends .posegraph/.data itself — strip if user picked the file."""
    for ext in ('.posegraph', '.data'):
        if path.endswith(ext):
            return path[: -len(ext)]
    return path


class MapManagerNode(Node):
    def __init__(self):
        super().__init__('map_manager')
        self._save_cli = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self._load_cli = self.create_client(DeserializePoseGraph, '/slam_toolbox/deserialize_map')
        self._reset_cli = self.create_client(Reset, '/slam_toolbox/reset')

    # ── Save (fixed path) ──────────────────────────────────────────────────────
    def save(self, callback):
        self._do_save(MAP_FILE, callback)

    # ── Save As (custom path) ──────────────────────────────────────────────────
    def save_as(self, path: str, callback):
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self._do_save(path, callback)

    def _do_save(self, path: str, callback):
        os.makedirs(MAP_DIR, exist_ok=True)
        if not self._save_cli.service_is_ready():
            callback(False, '/slam_toolbox/serialize_map service not available')
            return
        req = SerializePoseGraph.Request()
        req.filename = path
        fut = self._save_cli.call_async(req)
        fut.add_done_callback(lambda f: self._on_save_done(f, path, callback))

    def _on_save_done(self, future, path, callback):
        try:
            res = future.result()
            ok = res is not None and res.result == 0
            callback(ok, os.path.basename(path) if ok else f'error code: {res.result}')
        except Exception as exc:
            callback(False, str(exc))

    # ── Load ───────────────────────────────────────────────────────────────────
    def load(self, path: str, callback):
        threading.Thread(target=self._load_thread, args=(path, callback), daemon=True).start()

    def _load_thread(self, path: str, callback):
        if not self._load_cli.wait_for_service(timeout_sec=3.0):
            callback(False, '/slam_toolbox/deserialize_map service not available')
            return
        req = DeserializePoseGraph.Request()
        req.filename = path
        req.match_type = DeserializePoseGraph.Request.START_AT_FIRST_NODE
        fut = self._load_cli.call_async(req)
        deadline = time.time() + 10.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.05)
        if not fut.done():
            callback(False, 'Load timed out')
            return
        callback(True, os.path.basename(path))

    # ── Reset ──────────────────────────────────────────────────────────────────
    def restart_slam(self, callback):
        threading.Thread(target=self._restart_thread, args=(callback,), daemon=True).start()

    def _restart_thread(self, callback):
        if not self._reset_cli.wait_for_service(timeout_sec=3.0):
            callback(False, '/slam_toolbox/reset service not available')
            return
        req = Reset.Request()
        req.pause_new_measurements = False
        fut = self._reset_cli.call_async(req)
        deadline = time.time() + 6.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.05)
        if not fut.done():
            callback(False, 'Reset timed out')
            return
        try:
            res = fut.result()
            ok = res.result == Reset.Response.RESULT_SUCCESS
            callback(ok, 'Map cleared' if ok else f'Reset error: {res.result}')
        except Exception as exc:
            callback(False, str(exc))


class App:
    def __init__(self, node: MapManagerNode):
        self._node = node
        self._root = tk.Tk()
        self._root.title('Map Manager')
        self._root.resizable(False, False)
        self._build_ui()
        self._schedule_autosave()
        self._log(f'Ready  —  {MAP_FILE}')

    def _build_ui(self):
        outer = tk.Frame(self._root, padx=14, pady=14)
        outer.pack(fill='both', expand=True)

        self._log_box = tk.Text(
            outer, width=54, height=14, state='disabled',
            font=('Courier', 9), bg='#1a1a2e', fg='#d4d4d4',
            relief='flat', cursor='arrow',
        )
        self._log_box.pack(fill='both', expand=True)

        self._cvar = tk.StringVar(value=f'Auto-save: {AUTO_SAVE_INTERVAL}s')
        tk.Label(outer, textvariable=self._cvar, font=('Arial', 9), fg='#777') \
            .pack(pady=(6, 4))

        def btn(parent, text, color, hover, cmd, side='left', padx=(0, 6)):
            b = tk.Button(parent, text=text, width=18,
                          bg=color, fg='white', activebackground=hover,
                          font=('Arial', 10, 'bold'), relief='flat', pady=6,
                          command=cmd)
            b.pack(side=side, padx=padx)
            return b

        row1 = tk.Frame(outer)
        row1.pack(fill='x', pady=(0, 6))
        btn(row1, 'Save',     '#2563eb', '#1d4ed8', self._on_save)
        btn(row1, 'Save As',  '#0891b2', '#0e7490', self._on_save_as, padx=(0, 0))

        row2 = tk.Frame(outer)
        row2.pack(fill='x')
        btn(row2, 'Load Map', '#059669', '#047857', self._on_load)
        btn(row2, 'Clear',    '#dc2626', '#b91c1c', self._on_clear, padx=(0, 0))

    def _log(self, msg: str):
        stamp = datetime.now().strftime('%H:%M:%S')
        self._log_box.config(state='normal')
        self._log_box.insert('end', f'[{stamp}] {msg}\n')
        self._log_box.see('end')
        self._log_box.config(state='disabled')

    def _cb(self, ok, msg):
        self._root.after(0, self._log, ('OK  ' if ok else 'ERR ') + msg)

    # ── Save ──────────────────────────────────────────────────────────────────
    def _on_save(self):
        self._log('Saving...')
        self._node.save(self._cb)

    # ── Save As ───────────────────────────────────────────────────────────────
    def _on_save_as(self):
        path = filedialog.asksaveasfilename(
            parent=self._root,
            title='Save Map As',
            initialdir=MAP_DIR,
            initialfile='warehouse_map',
            defaultextension='',
            filetypes=[('slam_toolbox map', '*.posegraph'), ('All files', '*')],
        )
        if not path:
            return
        path = _strip_posegraph(path)
        self._log(f'Save As → {os.path.basename(path)}')
        self._node.save_as(path, self._cb)

    # ── Load ──────────────────────────────────────────────────────────────────
    def _on_load(self):
        path = filedialog.askopenfilename(
            parent=self._root,
            title='Load Map',
            initialdir=MAP_DIR,
            filetypes=[('slam_toolbox map', '*.posegraph'), ('All files', '*')],
        )
        if not path:
            return
        path = _strip_posegraph(path)
        self._log(f'Loading → {os.path.basename(path)}')
        self._node.load(path, self._cb)

    # ── Clear ─────────────────────────────────────────────────────────────────
    def _on_clear(self):
        files = glob.glob(MAP_FILE + '.*')
        file_info = f'{len(files)} file(s)' if files else 'no saved files'
        if not messagebox.askyesno(
            'Confirm Clear',
            f'Delete saved map and reset live map?\n({file_info})\n\nThis cannot be undone.',
            icon='warning', parent=self._root,
        ):
            return
        for f in files:
            os.remove(f)
        if files:
            self._log(f'Deleted: {len(files)} file(s)')
        self._log('Resetting map...')
        self._node.restart_slam(self._cb)

    # ── Auto-save countdown ───────────────────────────────────────────────────
    def _schedule_autosave(self, remaining: int = AUTO_SAVE_INTERVAL):
        if remaining > 0:
            self._cvar.set(f'Auto-save: {remaining}s')
            self._root.after(1000, self._schedule_autosave, remaining - 1)
        else:
            self._cvar.set('Saving...')
            self._log('[auto] Saving...')
            self._node.save(lambda ok, msg: self._root.after(0, self._autosave_done, ok, msg))

    def _autosave_done(self, ok: bool, msg: str):
        self._log(('[auto] OK  ' if ok else '[auto] ERR ') + msg)
        self._schedule_autosave()

    def run(self):
        self._root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = MapManagerNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = App(node)
    try:
        app.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
