#!/usr/bin/env python3
"""Map manager GUI with per-map annotations and simple selection tools."""

import glob
import json
import math
import os
import threading
import time
import tkinter as tk
import uuid
from dataclasses import dataclass
from datetime import datetime
from tkinter import filedialog, messagebox, simpledialog

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import DeserializePoseGraph, Reset, SerializePoseGraph
from nav2_msgs.action import NavigateToPose

from nav_msgs.msg import OccupancyGrid
from PIL import Image, ImageTk

MAP_DIR = os.path.expanduser('~/otonom_projeler/hikbot/maps')
MAP_FILE = os.path.join(MAP_DIR, 'warehouse_map')
AUTO_SAVE_INTERVAL = 60
ANNOTATION_SCHEMA = 2

WORKSPACE_WIDTH = 1000
WORKSPACE_HEIGHT = 700
VIEW_MARGIN = 28
GRID_STEP = 50


def _strip_posegraph(path: str) -> str:
    """slam_toolbox appends .posegraph/.data itself; strip if user picked one."""
    for ext in ('.posegraph', '.data'):
        if path.endswith(ext):
            return path[: -len(ext)]
    return path


def _annotation_path(map_path: str) -> str:
    return f'{_strip_posegraph(map_path)}.annotations.json'


def _map_sidecar_files(map_path: str) -> list[str]:
    return sorted(glob.glob(_strip_posegraph(map_path) + '.*'))


def _rotate_point(x: float, y: float, cx: float, cy: float, angle_deg: float) -> tuple[float, float]:
    angle = math.radians(angle_deg)
    dx = x - cx
    dy = y - cy
    return (
        cx + math.cos(angle) * dx - math.sin(angle) * dy,
        cy + math.sin(angle) * dx + math.cos(angle) * dy,
    )


def _point_in_polygon(x: float, y: float, polygon: list[tuple[float, float]]) -> bool:
    inside = False
    if len(polygon) < 3:
        return inside

    j = len(polygon) - 1
    for i, point in enumerate(polygon):
        xi, yi = point
        xj, yj = polygon[j]
        intersects = (yi > y) != (yj > y)
        if intersects:
            slope_x = (xj - xi) * (y - yi) / ((yj - yi) or 1e-9) + xi
            if x < slope_x:
                inside = not inside
        j = i
    return inside


@dataclass
class Annotation:
    id: str
    label: str
    x: float
    y: float
    width: float
    height: float
    angle: float = 0.0
    color: str = '#f59e0b'

    @classmethod
    def from_dict(cls, data: dict) -> 'Annotation':
        return cls(
            id=str(data.get('id') or uuid.uuid4().hex),
            label=str(data.get('label') or 'Annotation'),
            x=float(data.get('x', 0.0)),
            y=float(data.get('y', 0.0)),
            width=float(data.get('width', 1.0)),
            height=float(data.get('height', 1.0)),
            angle=float(data.get('angle', 0.0)),
            color=str(data.get('color') or '#f59e0b'),
        )

    def to_dict(self) -> dict:
        return {
            'id': self.id,
            'type': 'area',
            'label': self.label,
            'x': self.x,
            'y': self.y,
            'width': self.width,
            'height': self.height,
            'angle': self.angle,
            'color': self.color,
        }

    def corners(self) -> list[tuple[float, float]]:
        half_w = self.width / 2.0
        half_h = self.height / 2.0
        points = [
            (self.x - half_w, self.y - half_h),
            (self.x + half_w, self.y - half_h),
            (self.x + half_w, self.y + half_h),
            (self.x - half_w, self.y + half_h),
        ]
        return [_rotate_point(px, py, self.x, self.y, self.angle) for px, py in points]


class MapManagerNode(Node):
    def __init__(self):
        super().__init__('map_manager')
        self._save_cli = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self._load_cli = self.create_client(DeserializePoseGraph, '/slam_toolbox/deserialize_map')
        self._reset_cli = self.create_client(Reset, '/slam_toolbox/reset')
        self._nav_cli = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_sub = self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.latest_map: OccupancyGrid | None = None
        self.map_image: Image.Image | None = None
        self.on_map_update = None

    def _on_map(self, msg: OccupancyGrid):
        self.latest_map = msg
        try:
            self.map_image = self._occupancy_to_image(msg)
        except Exception:
            pass
        if self.on_map_update:
            self.on_map_update()

    @staticmethod
    def _occupancy_to_image(msg: OccupancyGrid) -> Image.Image:
        width = msg.info.width
        height = msg.info.height
        data = bytearray(width * height * 3)
        for y in range(height):
            src_row = (height - 1 - y) * width
            dst_row = y * width
            for x in range(width):
                val = msg.data[src_row + x]
                offset = (dst_row + x) * 3
                if val == -1:
                    data[offset : offset + 3] = bytes([128, 128, 128])
                elif val == 0:
                    data[offset : offset + 3] = bytes([255, 255, 255])
                elif val == 100:
                    data[offset : offset + 3] = bytes([0, 0, 0])
                else:
                    v = max(0, min(255, 255 - int(val * 2.55)))
                    data[offset : offset + 3] = bytes([v, v, v])
        return Image.frombytes('RGB', (width, height), bytes(data))

    def save(self, path: str, callback):
        self._do_save(_strip_posegraph(path), callback)

    def _do_save(self, path: str, callback):
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
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

    def load(self, path: str, callback):
        threading.Thread(target=self._load_thread, args=(_strip_posegraph(path), callback), daemon=True).start()

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
        try:
            fut.result()
            callback(True, os.path.basename(path))
        except Exception as exc:
            callback(False, str(exc))

    def navigate_to(self, x: float, y: float, start_callback, result_callback=None):
        threading.Thread(target=self._nav_thread, args=(x, y, start_callback, result_callback), daemon=True).start()

    def _nav_thread(self, x: float, y: float, start_callback, result_callback=None):
        if not self._nav_cli.wait_for_server(timeout_sec=3.0):
            start_callback(False, '/navigate_to_pose action not available')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        future = self._nav_cli.send_goal_async(goal)
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            start_callback(False, 'Goal send timed out')
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            start_callback(False, 'Goal rejected by Nav2')
            return
        start_callback(True, f'Navigating to ({x:.2f}, {y:.2f})')

        if result_callback:
            result_future = goal_handle.get_result_async()
            deadline = time.time() + 120.0
            while not result_future.done() and time.time() < deadline:
                time.sleep(0.1)
            if not result_future.done():
                result_callback(False, 'Navigation timed out')
                return
            try:
                result_future.result()
                result_callback(True, 'Arrived')
            except Exception as exc:
                result_callback(False, str(exc))

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
        self._root.minsize(1080, 620)

        self._current_map_path = _strip_posegraph(MAP_FILE)
        self._annotations: list[Annotation] = []
        self._selected_ids: set[str] = set()
        self._map_rotation = 0.0

        self._tool_mode = tk.StringVar(value='select')
        self._current_map_var = tk.StringVar()
        self._annotation_var = tk.StringVar()
        self._cvar = tk.StringVar(value=f'Auto-save: {AUTO_SAVE_INTERVAL}s')
        self._status_var = tk.StringVar(value='Ready')

        self._navigating_to_id: str | None = None
        self._arrived_at_id: str | None = None

        self._drag_start_world: tuple[float, float] | None = None
        self._last_drag_world: tuple[float, float] | None = None
        self._draft_item: int | None = None
        self._dragging_selection = False
        self._lasso_points: list[tuple[float, float]] = []
        self._lasso_item: int | None = None
        self._last_canvas_size = (0, 0)
        self._map_photo: ImageTk.PhotoImage | None = None
        self._last_map_info_hash = None
        self._ann_list_frame: tk.Frame | None = None
        self._ann_list_canvas: tk.Canvas | None = None

        self._zoom_level = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._pan_drag_prev = (0.0, 0.0)

        self._build_ui()
        self._node.on_map_update = lambda: self._root.after(0, self._redraw)
        self._load_annotations()
        self._schedule_autosave()
        self._root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._log(f'Ready - {self._current_map_path}')

    def _build_ui(self):
        outer = tk.Frame(self._root, padx=12, pady=12)
        outer.pack(fill='both', expand=True)

        left = tk.Frame(outer)
        left.pack(side='left', fill='both', expand=True, padx=(0, 12))

        tk.Label(left, textvariable=self._current_map_var, anchor='w', font=('Arial', 10, 'bold')).pack(
            fill='x', pady=(0, 6)
        )
        self._canvas = tk.Canvas(
            left,
            width=760,
            height=500,
            bg='#101418',
            highlightthickness=1,
            highlightbackground='#2f3a45',
            cursor='crosshair',
        )
        self._canvas.pack(fill='both', expand=True)
        tk.Label(left, textvariable=self._status_var, anchor='w', fg='#555').pack(fill='x', pady=(6, 0))

        self._canvas.bind('<Configure>', self._on_canvas_configure)
        self._canvas.bind('<ButtonPress-1>', self._on_canvas_press)
        self._canvas.bind('<B1-Motion>', self._on_canvas_drag)
        self._canvas.bind('<ButtonRelease-1>', self._on_canvas_release)
        self._canvas.bind('<MouseWheel>', self._on_mousewheel)
        self._canvas.bind('<Button-4>', self._on_mousewheel)
        self._canvas.bind('<Button-5>', self._on_mousewheel)
        self._canvas.bind('<ButtonPress-2>', self._on_pan_press)
        self._canvas.bind('<B2-Motion>', self._on_pan_drag)
        self._canvas.bind('<ButtonRelease-2>', self._on_pan_release)
        self._root.bind('<Escape>', lambda _event: self._clear_interaction())
        self._root.bind('<Delete>', lambda _event: self._delete_selected())

        right = tk.Frame(outer, width=280)
        right.pack(side='right', fill='y')
        right.pack_propagate(False)

        self._log_box = tk.Text(
            right,
            width=38,
            height=12,
            state='disabled',
            font=('Courier', 9),
            bg='#1a1a2e',
            fg='#d4d4d4',
            relief='flat',
            cursor='arrow',
        )
        self._log_box.pack(fill='x')

        tk.Label(right, textvariable=self._cvar, font=('Arial', 9), fg='#777').pack(pady=(6, 8))
        self._build_map_buttons(right)
        self._build_annotation_tools(right)
        self._build_annotation_list(right)
        tk.Label(right, textvariable=self._annotation_var, anchor='w', justify='left', fg='#555').pack(
            fill='x', pady=(8, 0)
        )

    def _build_map_buttons(self, parent):
        def btn(row, text, color, hover, cmd, side='left', padx=(0, 6), width=12):
            b = tk.Button(
                row,
                text=text,
                width=width,
                bg=color,
                fg='white',
                activebackground=hover,
                font=('Arial', 9, 'bold'),
                relief='flat',
                pady=5,
                command=cmd,
            )
            b.pack(side=side, padx=padx)
            return b

        row1 = tk.Frame(parent)
        row1.pack(fill='x', pady=(0, 6))
        btn(row1, 'Save', '#2563eb', '#1d4ed8', self._on_save)
        btn(row1, 'Save As', '#0891b2', '#0e7490', self._on_save_as, padx=(0, 0))

        row2 = tk.Frame(parent)
        row2.pack(fill='x', pady=(0, 12))
        btn(row2, 'Load Map', '#059669', '#047857', self._on_load)
        btn(row2, 'Clear', '#dc2626', '#b91c1c', self._on_clear, padx=(0, 0))

    def _build_annotation_tools(self, parent):
        tools = tk.LabelFrame(parent, text='Annotations', padx=8, pady=8)
        tools.pack(fill='x')

        for text, value in (
            ('Select', 'select'),
            ('Add Area', 'add'),
            ('Lasso', 'lasso'),
        ):
            tk.Radiobutton(
                tools,
                text=text,
                variable=self._tool_mode,
                value=value,
                command=self._clear_interaction,
                anchor='w',
            ).pack(fill='x')

        tk.Frame(tools, height=8).pack()

        row1 = tk.Frame(tools)
        row1.pack(fill='x', pady=(0, 6))
        tk.Button(row1, text='Rename', command=self._rename_selected, width=12).pack(side='left', padx=(0, 6))
        tk.Button(row1, text='Delete', command=self._delete_selected, width=12).pack(side='left')

        row3 = tk.Frame(tools)
        row3.pack(fill='x')
        tk.Button(row3, text='Map -90', command=lambda: self._rotate_map(-90.0), width=12).pack(
            side='left', padx=(0, 6)
        )
        tk.Button(row3, text='Map +90', command=lambda: self._rotate_map(90.0), width=12).pack(side='left')

    def _build_annotation_list(self, parent):
        lf = tk.LabelFrame(parent, text='Locations', padx=4, pady=4)
        lf.pack(fill='both', expand=True, pady=(8, 0))

        container = tk.Frame(lf)
        container.pack(fill='both', expand=True)

        self._ann_list_canvas = tk.Canvas(
            container, height=140, bg='#1a1a2e', highlightthickness=0
        )
        scrollbar = tk.Scrollbar(container, orient='vertical', command=self._ann_list_canvas.yview)
        self._ann_list_canvas.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side='right', fill='y')
        self._ann_list_canvas.pack(side='left', fill='both', expand=True)

        self._ann_list_frame = tk.Frame(self._ann_list_canvas, bg='#1a1a2e')
        win = self._ann_list_canvas.create_window((0, 0), window=self._ann_list_frame, anchor='nw')

        self._ann_list_frame.bind(
            '<Configure>',
            lambda e: self._ann_list_canvas.configure(scrollregion=self._ann_list_canvas.bbox('all')),
        )
        self._ann_list_canvas.bind(
            '<Configure>',
            lambda e: self._ann_list_canvas.itemconfig(win, width=e.width),
        )

    def _refresh_annotation_list(self):
        if self._ann_list_frame is None:
            return
        for widget in self._ann_list_frame.winfo_children():
            widget.destroy()
        for ann in self._annotations:
            row = tk.Frame(self._ann_list_frame, bg='#1a1a2e')
            row.pack(fill='x', pady=1, padx=2)
            tk.Label(
                row,
                text=ann.label,
                anchor='w',
                bg='#1a1a2e',
                fg='#d4d4d4',
                font=('Arial', 9),
            ).pack(side='left', fill='x', expand=True)

            if ann.id == self._arrived_at_id:
                btn_text = 'Arrived'
                btn_bg = '#0891b2'
                btn_hover = '#0e7490'
            elif ann.id == self._navigating_to_id:
                btn_text = f'Going to {ann.label}'
                btn_bg = '#d97706'
                btn_hover = '#b45309'
            else:
                btn_text = 'Go'
                btn_bg = '#059669'
                btn_hover = '#047857'

            tk.Button(
                row,
                text=btn_text,
                bg=btn_bg,
                fg='white',
                activebackground=btn_hover,
                font=('Arial', 8, 'bold'),
                relief='flat',
                pady=2,
                command=lambda a=ann: self._navigate_to_annotation(a),
            ).pack(side='right', padx=(4, 0))
        if self._ann_list_canvas is not None:
            self._ann_list_canvas.configure(scrollregion=self._ann_list_canvas.bbox('all'))

    def _navigate_to_annotation(self, ann: Annotation):
        if self._node.latest_map is None:
            self._log('ERR No map loaded — cannot navigate')
            return
        self._arrived_at_id = None
        self._navigating_to_id = ann.id
        self._refresh_annotation_list()
        self._log(f'Go -> {ann.label} ({ann.x:.2f}, {ann.y:.2f})')

        def on_start(ok, msg):
            self._root.after(0, self._cb, ok, msg)

        def on_result(ok, msg):
            self._root.after(0, self._on_nav_result, ok, msg, ann.id)

        self._node.navigate_to(ann.x, ann.y, on_start, on_result)

    def _log(self, msg: str):
        stamp = datetime.now().strftime('%H:%M:%S')
        self._log_box.config(state='normal')
        self._log_box.insert('end', f'[{stamp}] {msg}\n')
        self._log_box.see('end')
        self._log_box.config(state='disabled')

    def _cb(self, ok, msg):
        self._root.after(0, self._log, ('OK  ' if ok else 'ERR ') + msg)

    def _on_nav_result(self, ok: bool, msg: str, annotation_id: str):
        self._log(('OK  ' if ok else 'ERR ') + msg)
        self._navigating_to_id = None
        if ok:
            self._arrived_at_id = annotation_id
        self._refresh_annotation_list()

    def _set_current_map_label(self):
        self._current_map_var.set(f'Map: {os.path.basename(self._current_map_path)}')
        self._annotation_var.set(
            f'Annotation file:\n{os.path.basename(_annotation_path(self._current_map_path))}'
        )
        self._root.title(f'Map Manager - {os.path.basename(self._current_map_path)}')

    def _update_status(self, text: str | None = None):
        selected = len(self._selected_ids)
        base = f'{len(self._annotations)} annotation(s), {selected} selected, map rotation {self._map_rotation:.0f} deg'
        self._status_var.set(f'{base} - {text}' if text else base)

    def _annotation_payload(self) -> dict:
        return {
            'schema': ANNOTATION_SCHEMA,
            'coord': 'world',
            'map_path': self._current_map_path,
            'map_rotation': self._map_rotation,
            'workspace': {
                'width': WORKSPACE_WIDTH,
                'height': WORKSPACE_HEIGHT,
            },
            'annotations': [annotation.to_dict() for annotation in self._annotations],
        }

    def _save_annotations(self):
        path = _annotation_path(self._current_map_path)
        has_state = bool(self._annotations) or (self._map_rotation % 360.0) != 0.0
        if not has_state:
            if os.path.exists(path):
                os.remove(path)
            return
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        with open(path, 'w', encoding='utf-8') as handle:
            json.dump(self._annotation_payload(), handle, indent=2)

    def _load_annotations(self):
        self._set_current_map_label()
        path = _annotation_path(self._current_map_path)
        self._annotations = []
        self._selected_ids.clear()
        self._map_rotation = 0.0

        if os.path.exists(path):
            try:
                with open(path, 'r', encoding='utf-8') as handle:
                    payload = json.load(handle)
                schema = int(payload.get('schema', 1))
                self._map_rotation = float(payload.get('map_rotation', 0.0))
                self._annotations = [
                    Annotation.from_dict(item)
                    for item in payload.get('annotations', [])
                    if isinstance(item, dict)
                ]
                if schema < 2:
                    self._migrate_schema1_annotations()
                self._log(f'Loaded annotations: {os.path.basename(path)}')
            except (OSError, ValueError, TypeError) as exc:
                self._log(f'ERR annotation load failed: {exc}')

        self._redraw()
        self._update_status()
        self._refresh_annotation_list()

    def _migrate_schema1_annotations(self):
        """Convert workspace-pixel coords (schema 1) to world meters (schema 2)."""
        if self._node.latest_map is None:
            self._log('WARN: schema 1 annotations loaded without map - positions approximate')
            return
        info = self._node.latest_map.info
        fit = min(WORKSPACE_WIDTH / info.width, WORKSPACE_HEIGHT / info.height)
        for ann in self._annotations:
            try:
                ann.x, ann.y = self._workspace_to_world(ann.x, ann.y)
                ann.width = ann.width / fit * info.resolution
                ann.height = ann.height / fit * info.resolution
            except Exception:
                pass
        self._log('Migrated schema 1 annotations to world coordinates')

    def _switch_to_map(self, path: str, load_annotations: bool):
        try:
            self._save_annotations()
        except OSError as exc:
            self._log(f'ERR annotation save failed: {exc}')
        self._current_map_path = _strip_posegraph(path)
        if load_annotations:
            self._load_annotations()
        else:
            self._set_current_map_label()
            self._save_annotations()
            self._redraw()
            self._update_status()

    def _on_canvas_configure(self, event):
        size = (event.width, event.height)
        if size != self._last_canvas_size:
            self._last_canvas_size = size
            self._map_photo = None
            self._redraw()

    def _img_fit_scale(self) -> float | None:
        """Workspace pixels per map image pixel. None if no map."""
        m = self._node.latest_map
        if m is None or m.info.width == 0 or m.info.height == 0:
            return None
        return min(WORKSPACE_WIDTH / m.info.width, WORKSPACE_HEIGHT / m.info.height)

    def _world_to_workspace(self, wx: float, wy: float) -> tuple[float, float]:
        info = self._node.latest_map.info
        fit = min(WORKSPACE_WIDTH / info.width, WORKSPACE_HEIGHT / info.height)
        px = (wx - info.origin.position.x) / info.resolution
        py = info.height - (wy - info.origin.position.y) / info.resolution
        return (
            WORKSPACE_WIDTH / 2.0 + (px - info.width / 2.0) * fit,
            WORKSPACE_HEIGHT / 2.0 + (py - info.height / 2.0) * fit,
        )

    def _workspace_to_world(self, ws_x: float, ws_y: float) -> tuple[float, float]:
        info = self._node.latest_map.info
        fit = min(WORKSPACE_WIDTH / info.width, WORKSPACE_HEIGHT / info.height)
        px = (ws_x - WORKSPACE_WIDTH / 2.0) / fit + info.width / 2.0
        py = (ws_y - WORKSPACE_HEIGHT / 2.0) / fit + info.height / 2.0
        wx = info.origin.position.x + px * info.resolution
        wy = info.origin.position.y + (info.height - py) * info.resolution
        return wx, wy

    def _canvas_fit_scale(self) -> float:
        """Canvas pixels per map image pixel (for current canvas size)."""
        info = self._node.latest_map.info
        usable_w = max(1, self._canvas.winfo_width() - VIEW_MARGIN * 2)
        usable_h = max(1, self._canvas.winfo_height() - VIEW_MARGIN * 2)
        return min(usable_w / info.width, usable_h / info.height) * self._zoom_level

    def _world_to_canvas(self, wx: float, wy: float) -> tuple[float, float]:
        """World metres → canvas pixels using PIL-convention rotation (CCW positive)."""
        info = self._node.latest_map.info
        fit = self._canvas_fit_scale()
        px = (wx - info.origin.position.x) / info.resolution
        py = info.height - (wy - info.origin.position.y) / info.resolution
        dx = (px - info.width / 2.0) * fit
        dy = (py - info.height / 2.0) * fit
        angle = math.radians(self._map_rotation)
        # PIL rotate(angle) = CCW visual in y-down coords:
        #   new_dx =  cos(a)*dx + sin(a)*dy
        #   new_dy = -sin(a)*dx + cos(a)*dy
        rx = math.cos(angle) * dx + math.sin(angle) * dy
        ry = -math.sin(angle) * dx + math.cos(angle) * dy
        view_cx, view_cy, _ = self._view_params()
        return (view_cx + rx + self._pan_x, view_cy + ry + self._pan_y)

    def _canvas_to_world(self, cx: float, cy: float) -> tuple[float, float]:
        """Canvas pixels → world metres (inverse of _world_to_canvas)."""
        info = self._node.latest_map.info
        fit = self._canvas_fit_scale()
        view_cx, view_cy, _ = self._view_params()
        rx = cx - view_cx - self._pan_x
        ry = cy - view_cy - self._pan_y
        angle = math.radians(self._map_rotation)
        # Inverse of PIL CCW = CW = standard _rotate_point convention:
        dx = math.cos(angle) * rx - math.sin(angle) * ry
        dy = math.sin(angle) * rx + math.cos(angle) * ry
        px = dx / fit + info.width / 2.0
        py = dy / fit + info.height / 2.0
        wx = info.origin.position.x + px * info.resolution
        wy = info.origin.position.y + (info.height - py) * info.resolution
        return wx, wy

    def _view_params(self) -> tuple[float, float, float]:
        width = max(1, self._canvas.winfo_width())
        height = max(1, self._canvas.winfo_height())
        usable_w = max(1, width - VIEW_MARGIN * 2)
        usable_h = max(1, height - VIEW_MARGIN * 2)
        scale = min(usable_w / WORKSPACE_WIDTH, usable_h / WORKSPACE_HEIGHT)
        return width / 2.0, height / 2.0, scale

    def _map_to_canvas(self, x: float, y: float) -> tuple[float, float]:
        view_cx, view_cy, scale = self._view_params()
        map_cx = WORKSPACE_WIDTH / 2.0
        map_cy = WORKSPACE_HEIGHT / 2.0
        rx, ry = _rotate_point(x, y, map_cx, map_cy, self._map_rotation)
        return (
            view_cx + (rx - map_cx) * scale + self._pan_x,
            view_cy + (ry - map_cy) * scale + self._pan_y,
        )

    def _canvas_to_map(self, x: float, y: float) -> tuple[float, float]:
        view_cx, view_cy, scale = self._view_params()
        map_cx = WORKSPACE_WIDTH / 2.0
        map_cy = WORKSPACE_HEIGHT / 2.0
        rx = map_cx + (x - view_cx - self._pan_x) / scale
        ry = map_cy + (y - view_cy - self._pan_y) / scale
        return _rotate_point(rx, ry, map_cx, map_cy, -self._map_rotation)

    def _redraw(self):
        self._canvas.delete('all')
        self._draw_map_image()
        self._draw_workspace()
        for annotation in self._annotations:
            self._draw_annotation(annotation)

    def _draw_map_image(self):
        node = self._node
        if node.map_image is None:
            return

        pil_img = node.map_image
        img_w = pil_img.width
        img_h = pil_img.height
        if img_w <= 0 or img_h <= 0:
            return

        canvas_w = self._canvas.winfo_width()
        canvas_h = self._canvas.winfo_height()
        usable_w = max(1, canvas_w - VIEW_MARGIN * 2)
        usable_h = max(1, canvas_h - VIEW_MARGIN * 2)
        # Scale to canvas pixels (not workspace pixels) so alignment matches _world_to_canvas
        fit = min(usable_w / img_w, usable_h / img_h) * self._zoom_level
        draw_w = max(1, int(img_w * fit))
        draw_h = max(1, int(img_h * fit))

        current_hash = (img_w, img_h, self._map_rotation, canvas_w, canvas_h, self._zoom_level)
        if current_hash != self._last_map_info_hash or self._map_photo is None:
            # Resize first → then rotate: avoids aspect-ratio distortion on non-square maps
            resized = pil_img.resize((draw_w, draw_h), Image.NEAREST)
            rotated = resized.rotate(self._map_rotation, resample=Image.NEAREST, expand=True)
            self._map_photo = ImageTk.PhotoImage(rotated)
            self._last_map_info_hash = current_hash

        view_cx, view_cy, _ = self._view_params()
        self._canvas.create_image(
            view_cx + self._pan_x, view_cy + self._pan_y,
            image=self._map_photo, anchor='center', tags=('map_bg',),
        )

    def _workspace_corners(self):
        return [
            self._map_to_canvas(0, 0),
            self._map_to_canvas(WORKSPACE_WIDTH, 0),
            self._map_to_canvas(WORKSPACE_WIDTH, WORKSPACE_HEIGHT),
            self._map_to_canvas(0, WORKSPACE_HEIGHT),
        ]

    def _draw_workspace(self):
        corners = self._workspace_corners()
        self._canvas.create_polygon(corners, fill='', outline='#607083', width=2)

        for x in range(0, WORKSPACE_WIDTH + 1, GRID_STEP):
            self._canvas.create_line(
                *self._map_to_canvas(x, 0),
                *self._map_to_canvas(x, WORKSPACE_HEIGHT),
                fill='#25313d',
            )
        for y in range(0, WORKSPACE_HEIGHT + 1, GRID_STEP):
            self._canvas.create_line(
                *self._map_to_canvas(0, y),
                *self._map_to_canvas(WORKSPACE_WIDTH, y),
                fill='#25313d',
            )

        self._canvas.create_line(
            *self._map_to_canvas(WORKSPACE_WIDTH / 2, 0),
            *self._map_to_canvas(WORKSPACE_WIDTH / 2, WORKSPACE_HEIGHT),
            fill='#3b4c5d',
            width=2,
        )
        self._canvas.create_line(
            *self._map_to_canvas(0, WORKSPACE_HEIGHT / 2),
            *self._map_to_canvas(WORKSPACE_WIDTH, WORKSPACE_HEIGHT / 2),
            fill='#3b4c5d',
            width=2,
        )

    def _annotation_canvas_center(self, annotation: Annotation) -> tuple[float, float] | None:
        if self._node.latest_map is None:
            return None
        return self._world_to_canvas(annotation.x, annotation.y)

    def _annotation_canvas_corners(self, annotation: Annotation) -> list[tuple[float, float]] | None:
        if self._node.latest_map is None:
            return None
        half_w = annotation.width / 2.0
        half_h = annotation.height / 2.0
        local = [(-half_w, -half_h), (half_w, -half_h), (half_w, half_h), (-half_w, half_h)]
        world_corners = [
            _rotate_point(annotation.x + dx, annotation.y + dy, annotation.x, annotation.y, annotation.angle)
            for dx, dy in local
        ]
        return [self._world_to_canvas(wx, wy) for wx, wy in world_corners]

    def _draw_annotation(self, annotation: Annotation):
        canvas_points = self._annotation_canvas_corners(annotation)
        if canvas_points is None:
            return
        selected = annotation.id in self._selected_ids
        flat_points = [coord for point in canvas_points for coord in point]
        self._canvas.create_polygon(
            flat_points,
            fill=annotation.color,
            stipple='gray25',
            outline='#ffffff' if selected else annotation.color,
            width=3 if selected else 2,
            tags=('annotation', annotation.id),
        )
        cx, cy = self._annotation_canvas_center(annotation)
        self._canvas.create_text(
            cx,
            cy,
            text=annotation.label,
            fill='#ffffff',
            font=('Arial', 10, 'bold'),
            tags=('annotation', annotation.id),
        )

    def _annotation_at(self, canvas_x: float, canvas_y: float) -> Annotation | None:
        for annotation in reversed(self._annotations):
            polygon = self._annotation_canvas_corners(annotation)
            if polygon is not None and _point_in_polygon(canvas_x, canvas_y, polygon):
                return annotation
        return None

    def _on_mousewheel(self, event):
        if self._node.latest_map is None:
            return
        old_zoom = self._zoom_level
        if event.num == 4 or (hasattr(event, 'delta') and event.delta > 0):
            self._zoom_level = min(10.0, old_zoom * 1.12)
        elif event.num == 5 or (hasattr(event, 'delta') and event.delta < 0):
            self._zoom_level = max(0.12, old_zoom / 1.12)
        else:
            return
        if self._zoom_level == old_zoom:
            return
        view_cx, view_cy, _ = self._view_params()
        ratio = self._zoom_level / old_zoom
        self._pan_x = (1.0 - ratio) * (event.x - view_cx) + ratio * self._pan_x
        self._pan_y = (1.0 - ratio) * (event.y - view_cy) + ratio * self._pan_y
        self._map_photo = None
        self._redraw()

    def _on_pan_press(self, event):
        self._pan_drag_prev = (event.x, event.y)

    def _on_pan_drag(self, event):
        dx = event.x - self._pan_drag_prev[0]
        dy = event.y - self._pan_drag_prev[1]
        self._pan_drag_prev = (event.x, event.y)
        self._pan_x += dx
        self._pan_y += dy
        self._canvas.move('all', dx, dy)

    def _on_pan_release(self, _event):
        self._redraw()

    def _on_canvas_press(self, event):
        self._clear_draft_items()
        mode = self._tool_mode.get()
        if mode == 'add':
            if self._node.latest_map is not None:
                self._drag_start_world = self._canvas_to_world(event.x, event.y)
            self._update_status('drag to create area')
            return
        if mode == 'lasso':
            self._lasso_points = [(event.x, event.y)]
            self._lasso_item = self._canvas.create_line(event.x, event.y, event.x, event.y, fill='#38bdf8', width=2)
            self._update_status('lasso selecting')
            return

        annotation = self._annotation_at(event.x, event.y)
        shift_pressed = bool(event.state & 0x0001)
        if annotation is None:
            if not shift_pressed:
                self._selected_ids.clear()
                self._redraw()
            self._dragging_selection = False
            self._update_status()
            return

        if shift_pressed:
            if annotation.id in self._selected_ids:
                self._selected_ids.remove(annotation.id)
            else:
                self._selected_ids.add(annotation.id)
        elif annotation.id not in self._selected_ids:
            self._selected_ids = {annotation.id}

        self._dragging_selection = True
        if self._node.latest_map is not None:
            self._last_drag_world = self._canvas_to_world(event.x, event.y)
        self._redraw()
        self._update_status('drag selection')

    def _on_canvas_drag(self, event):
        mode = self._tool_mode.get()
        if mode == 'add' and self._drag_start_world is not None:
            if self._node.latest_map is not None:
                self._draw_draft_area(self._drag_start_world, self._canvas_to_world(event.x, event.y))
            return
        if mode == 'lasso' and self._lasso_points:
            point = (event.x, event.y)
            last = self._lasso_points[-1]
            if math.hypot(point[0] - last[0], point[1] - last[1]) >= 3.0:
                self._lasso_points.append(point)
                if self._lasso_item is not None:
                    self._canvas.coords(self._lasso_item, *[coord for p in self._lasso_points for coord in p])
            return
        if mode == 'select' and self._dragging_selection and self._last_drag_world is not None:
            if self._node.latest_map is not None:
                current = self._canvas_to_world(event.x, event.y)
                dx = current[0] - self._last_drag_world[0]
                dy = current[1] - self._last_drag_world[1]
                for annotation in self._annotations:
                    if annotation.id in self._selected_ids:
                        annotation.x += dx
                        annotation.y += dy
                self._last_drag_world = current
                self._redraw()

    def _on_canvas_release(self, event):
        mode = self._tool_mode.get()
        if mode == 'add' and self._drag_start_world is not None:
            start = self._drag_start_world
            self._drag_start_world = None
            self._clear_draft_items()
            if self._node.latest_map is not None:
                self._create_annotation_from_drag(start, self._canvas_to_world(event.x, event.y))
            return
        if mode == 'lasso':
            self._finish_lasso()
            return
        if mode == 'select' and self._dragging_selection:
            self._dragging_selection = False
            self._last_drag_world = None
            self._persist_annotations('Moved selection')

    def _draw_draft_area(self, start_world: tuple[float, float], end_world: tuple[float, float]):
        self._clear_draft_items()
        x1, y1 = start_world
        x2, y2 = end_world
        cx_w = (x1 + x2) / 2.0
        cy_w = (y1 + y2) / 2.0
        half_w = abs(x2 - x1) / 2.0
        half_h = abs(y2 - y1) / 2.0
        world_corners = [
            (cx_w - half_w, cy_w - half_h),
            (cx_w + half_w, cy_w - half_h),
            (cx_w + half_w, cy_w + half_h),
            (cx_w - half_w, cy_w + half_h),
        ]
        canvas_corners = [self._world_to_canvas(wx, wy) for wx, wy in world_corners]
        self._draft_item = self._canvas.create_polygon(
            [coord for point in canvas_corners for coord in point],
            fill='#f59e0b',
            stipple='gray50',
            outline='#ffffff',
            width=2,
            dash=(4, 2),
        )

    def _create_annotation_from_drag(self, start_world: tuple[float, float], end_world: tuple[float, float]):
        x1, y1 = start_world  # world metres
        x2, y2 = end_world
        width = abs(x2 - x1)
        height = abs(y2 - y1)
        info = self._node.latest_map.info
        min_size = info.resolution * 2  # at least 2 map cells
        if width < min_size or height < min_size:
            self._update_status('area too small')
            return

        label = simpledialog.askstring('Annotation', 'Label:', parent=self._root)
        if label is None:
            label = f'Annotation {len(self._annotations) + 1}'
        label = label.strip() or f'Annotation {len(self._annotations) + 1}'

        annotation = Annotation(
            id=uuid.uuid4().hex,
            label=label,
            x=(x1 + x2) / 2.0,
            y=(y1 + y2) / 2.0,
            width=width,
            height=height,
        )
        self._annotations.append(annotation)
        self._selected_ids = {annotation.id}
        self._persist_annotations('Annotation added')

    def _finish_lasso(self):
        if self._lasso_item is not None:
            self._canvas.delete(self._lasso_item)
            self._lasso_item = None
        polygon = self._lasso_points
        self._lasso_points = []
        if len(polygon) < 3:
            self._update_status()
            return

        self._selected_ids = {
            annotation.id
            for annotation in self._annotations
            if (c := self._annotation_canvas_center(annotation)) is not None
            and _point_in_polygon(*c, polygon)
        }
        self._redraw()
        self._update_status('lasso complete')
        self._log(f'Lasso selected {len(self._selected_ids)} annotation(s)')

    def _clear_draft_items(self):
        if self._draft_item is not None:
            self._canvas.delete(self._draft_item)
            self._draft_item = None

    def _clear_interaction(self):
        self._drag_start_world = None
        self._last_drag_world = None
        self._dragging_selection = False
        self._clear_draft_items()
        if self._lasso_item is not None:
            self._canvas.delete(self._lasso_item)
            self._lasso_item = None
        self._lasso_points = []
        self._update_status()

    def _persist_annotations(self, message: str):
        try:
            self._save_annotations()
            self._log(message)
        except OSError as exc:
            self._log(f'ERR annotation save failed: {exc}')
        self._redraw()
        self._update_status()
        self._refresh_annotation_list()

    def _selected_annotations(self) -> list[Annotation]:
        return [annotation for annotation in self._annotations if annotation.id in self._selected_ids]

    def _rename_selected(self):
        selected = self._selected_annotations()
        if not selected:
            self._update_status('no selection')
            return
        initial = selected[0].label if len(selected) == 1 else ''
        label = simpledialog.askstring('Rename Annotation', 'New label:', initialvalue=initial, parent=self._root)
        if label is None:
            return
        label = label.strip()
        if not label:
            return
        for annotation in selected:
            annotation.label = label
        self._persist_annotations('Renamed annotation(s)')

    def _delete_selected(self):
        if not self._selected_ids:
            self._update_status('no selection')
            return
        count = len(self._selected_ids)
        self._annotations = [annotation for annotation in self._annotations if annotation.id not in self._selected_ids]
        self._selected_ids.clear()
        self._persist_annotations(f'Deleted {count} annotation(s)')

    def _rotate_selection(self, delta_deg: float):
        selected = self._selected_annotations()
        if not selected:
            self._update_status('no selection')
            return
        for annotation in selected:
            annotation.angle = (annotation.angle + delta_deg) % 360.0
        self._persist_annotations(f'Rotated selection {delta_deg:+.0f} deg')

    def _rotate_map(self, delta_deg: float):
        self._map_rotation = (self._map_rotation + delta_deg) % 360.0
        self._persist_annotations(f'Rotated map view {delta_deg:+.0f} deg')

    def _on_save(self):
        self._log('Saving...')
        try:
            self._save_annotations()
        except OSError as exc:
            self._log(f'ERR annotation save failed: {exc}')
        self._node.save(self._current_map_path, self._cb)

    def _on_save_as(self):
        path = filedialog.asksaveasfilename(
            parent=self._root,
            title='Save Map As',
            initialdir=MAP_DIR,
            initialfile=os.path.basename(self._current_map_path),
            defaultextension='',
            filetypes=[('slam_toolbox map', '*.posegraph'), ('All files', '*')],
        )
        if not path:
            return
        path = _strip_posegraph(path)
        self._log(f'Save As -> {os.path.basename(path)}')
        self._node.save(path, lambda ok, msg: self._root.after(0, self._after_save_as, ok, msg, path))

    def _after_save_as(self, ok: bool, msg: str, path: str):
        self._log(('OK  ' if ok else 'ERR ') + msg)
        if ok:
            self._switch_to_map(path, load_annotations=False)
            self._log(f'Active map -> {os.path.basename(path)}')

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
        self._log(f'Loading -> {os.path.basename(path)}')
        self._node.load(path, lambda ok, msg: self._root.after(0, self._after_load, ok, msg, path))

    def _after_load(self, ok: bool, msg: str, path: str):
        self._log(('OK  ' if ok else 'ERR ') + msg)
        if ok:
            self._switch_to_map(path, load_annotations=True)

    def _on_clear(self):
        files = _map_sidecar_files(self._current_map_path)
        file_info = f'{len(files)} file(s)' if files else 'no saved files'
        if not messagebox.askyesno(
            'Confirm Clear',
            f'Delete active map and reset live map?\n{self._current_map_path}\n({file_info})\n\nThis cannot be undone.',
            icon='warning',
            parent=self._root,
        ):
            return
        deleted = 0
        for path in files:
            try:
                os.remove(path)
                deleted += 1
            except OSError as exc:
                self._log(f'ERR delete failed: {os.path.basename(path)} - {exc}')
        self._annotations = []
        self._selected_ids.clear()
        self._map_rotation = 0.0
        self._redraw()
        self._update_status()
        if deleted:
            self._log(f'Deleted: {deleted} file(s)')
        self._log('Resetting map...')
        self._node.restart_slam(self._cb)

    def _schedule_autosave(self, remaining: int = AUTO_SAVE_INTERVAL):
        if remaining > 0:
            self._cvar.set(f'Auto-save: {remaining}s')
            self._root.after(1000, self._schedule_autosave, remaining - 1)
        else:
            self._cvar.set('Saving...')
            self._log('[auto] Saving...')
            try:
                self._save_annotations()
            except OSError as exc:
                self._log(f'[auto] ERR annotation save failed: {exc}')
            self._node.save(
                self._current_map_path,
                lambda ok, msg: self._root.after(0, self._autosave_done, ok, msg),
            )

    def _autosave_done(self, ok: bool, msg: str):
        self._log(('[auto] OK  ' if ok else '[auto] ERR ') + msg)
        self._schedule_autosave()

    def _on_close(self):
        try:
            self._save_annotations()
        except OSError as exc:
            self._log(f'ERR annotation save failed: {exc}')
        self._root.destroy()

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
