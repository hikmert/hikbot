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
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import DeserializePoseGraph, Reset, SerializePoseGraph

from nav_msgs.msg import OccupancyGrid
from PIL import Image, ImageTk

MAP_DIR = os.path.expanduser('~/otonom_projeler/hikbot/maps')
MAP_FILE = os.path.join(MAP_DIR, 'warehouse_map')
AUTO_SAVE_INTERVAL = 60
ANNOTATION_SCHEMA = 1

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
            x=float(data.get('x', WORKSPACE_WIDTH / 2)),
            y=float(data.get('y', WORKSPACE_HEIGHT / 2)),
            width=max(1.0, float(data.get('width', 80.0))),
            height=max(1.0, float(data.get('height', 60.0))),
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
        for i, val in enumerate(msg.data):
            offset = i * 3
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

        self._drag_start_map: tuple[float, float] | None = None
        self._last_drag_map: tuple[float, float] | None = None
        self._draft_item: int | None = None
        self._dragging_selection = False
        self._lasso_points: list[tuple[float, float]] = []
        self._lasso_item: int | None = None
        self._last_canvas_size = (0, 0)
        self._map_photo: ImageTk.PhotoImage | None = None
        self._last_map_info_hash = None

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

        row2 = tk.Frame(tools)
        row2.pack(fill='x', pady=(0, 6))
        tk.Button(row2, text='Sel -15', command=lambda: self._rotate_selection(-15.0), width=12).pack(
            side='left', padx=(0, 6)
        )
        tk.Button(row2, text='Sel +15', command=lambda: self._rotate_selection(15.0), width=12).pack(side='left')

        row3 = tk.Frame(tools)
        row3.pack(fill='x')
        tk.Button(row3, text='Map -90', command=lambda: self._rotate_map(-90.0), width=12).pack(
            side='left', padx=(0, 6)
        )
        tk.Button(row3, text='Map +90', command=lambda: self._rotate_map(90.0), width=12).pack(side='left')

    def _log(self, msg: str):
        stamp = datetime.now().strftime('%H:%M:%S')
        self._log_box.config(state='normal')
        self._log_box.insert('end', f'[{stamp}] {msg}\n')
        self._log_box.see('end')
        self._log_box.config(state='disabled')

    def _cb(self, ok, msg):
        self._root.after(0, self._log, ('OK  ' if ok else 'ERR ') + msg)

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
                self._map_rotation = float(payload.get('map_rotation', 0.0))
                self._annotations = [
                    Annotation.from_dict(item)
                    for item in payload.get('annotations', [])
                    if isinstance(item, dict)
                ]
                self._log(f'Loaded annotations: {os.path.basename(path)}')
            except (OSError, ValueError, TypeError) as exc:
                self._log(f'ERR annotation load failed: {exc}')

        self._redraw()
        self._update_status()

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
            view_cx + (rx - map_cx) * scale,
            view_cy + (ry - map_cy) * scale,
        )

    def _canvas_to_map(self, x: float, y: float) -> tuple[float, float]:
        view_cx, view_cy, scale = self._view_params()
        map_cx = WORKSPACE_WIDTH / 2.0
        map_cy = WORKSPACE_HEIGHT / 2.0
        rx = map_cx + (x - view_cx) / scale
        ry = map_cy + (y - view_cy) / scale
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
        ws_w = WORKSPACE_WIDTH
        ws_h = WORKSPACE_HEIGHT

        img_w = pil_img.width
        img_h = pil_img.height
        if img_w <= 0 or img_h <= 0:
            return

        scale_x = ws_w / img_w
        scale_y = ws_h / img_h
        fit_scale = min(scale_x, scale_y)

        draw_w = int(img_w * fit_scale)
        draw_h = int(img_h * fit_scale)

        offset_x = (ws_w - draw_w) / 2
        offset_y = (ws_h - draw_h) / 2

        current_hash = (node.latest_map.info.width, node.latest_map.info.height, self._map_rotation)
        if current_hash != self._last_map_info_hash or self._map_photo is None:
            rotated = pil_img.rotate(self._map_rotation, resample=Image.NEAREST, expand=True)
            resized = rotated.resize((draw_w, draw_h), Image.NEAREST)
            self._map_photo = ImageTk.PhotoImage(resized)
            self._last_map_info_hash = current_hash

        corners = self._workspace_corners()
        cx = sum(p[0] for p in corners) / len(corners)
        cy = sum(p[1] for p in corners) / len(corners)
        self._canvas.create_image(cx, cy, image=self._map_photo, anchor='center', tags=('map_bg',))

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

    def _annotation_canvas_center(self, annotation: Annotation) -> tuple[float, float]:
        view_cx, view_cy, scale = self._view_params()
        ws_cx = WORKSPACE_WIDTH / 2.0
        ws_cy = WORKSPACE_HEIGHT / 2.0
        rx, ry = _rotate_point(annotation.x, annotation.y, ws_cx, ws_cy, self._map_rotation)
        return (
            view_cx + (rx - ws_cx) * scale,
            view_cy + (ry - ws_cy) * scale,
        )

    def _annotation_canvas_corners(self, annotation: Annotation) -> list[tuple[float, float]]:
        center_cx, center_cy = self._annotation_canvas_center(annotation)
        _view_cx, _view_cy, scale = self._view_params()
        corners_ws = annotation.corners()
        rel = [(x - annotation.x, y - annotation.y) for x, y in corners_ws]
        rel_rot = [_rotate_point(rx, ry, 0, 0, self._map_rotation) for rx, ry in rel]
        return [(center_cx + rx * scale, center_cy + ry * scale) for rx, ry in rel_rot]

    def _draw_annotation(self, annotation: Annotation):
        canvas_points = self._annotation_canvas_corners(annotation)
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
            if _point_in_polygon(canvas_x, canvas_y, polygon):
                return annotation
        return None

    def _on_canvas_press(self, event):
        self._clear_draft_items()
        mode = self._tool_mode.get()
        if mode == 'add':
            self._drag_start_map = self._canvas_to_map(event.x, event.y)
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
        self._last_drag_map = self._canvas_to_map(event.x, event.y)
        self._redraw()
        self._update_status('drag selection')

    def _on_canvas_drag(self, event):
        mode = self._tool_mode.get()
        if mode == 'add' and self._drag_start_map is not None:
            self._draw_draft_area(self._drag_start_map, self._canvas_to_map(event.x, event.y))
            return
        if mode == 'lasso' and self._lasso_points:
            point = (event.x, event.y)
            last = self._lasso_points[-1]
            if math.hypot(point[0] - last[0], point[1] - last[1]) >= 3.0:
                self._lasso_points.append(point)
                if self._lasso_item is not None:
                    self._canvas.coords(self._lasso_item, *[coord for p in self._lasso_points for coord in p])
            return
        if mode == 'select' and self._dragging_selection and self._last_drag_map is not None:
            current = self._canvas_to_map(event.x, event.y)
            dx = current[0] - self._last_drag_map[0]
            dy = current[1] - self._last_drag_map[1]
            for annotation in self._annotations:
                if annotation.id in self._selected_ids:
                    annotation.x += dx
                    annotation.y += dy
            self._last_drag_map = current
            self._redraw()

    def _on_canvas_release(self, event):
        mode = self._tool_mode.get()
        if mode == 'add' and self._drag_start_map is not None:
            start = self._drag_start_map
            end = self._canvas_to_map(event.x, event.y)
            self._drag_start_map = None
            self._clear_draft_items()
            self._create_annotation_from_drag(start, end)
            return
        if mode == 'lasso':
            self._finish_lasso()
            return
        if mode == 'select' and self._dragging_selection:
            self._dragging_selection = False
            self._last_drag_map = None
            self._persist_annotations('Moved selection')

    def _draw_draft_area(self, start: tuple[float, float], end: tuple[float, float]):
        self._clear_draft_items()
        x1, y1 = start
        x2, y2 = end
        cx_ws = (x1 + x2) / 2.0
        cy_ws = (y1 + y2) / 2.0
        half_w = abs(x2 - x1) / 2.0
        half_h = abs(y2 - y1) / 2.0
        corners_ws = [
            (cx_ws - half_w, cy_ws - half_h),
            (cx_ws + half_w, cy_ws - half_h),
            (cx_ws + half_w, cy_ws + half_h),
            (cx_ws - half_w, cy_ws + half_h),
        ]
        view_cx, view_cy, scale = self._view_params()
        ws_cx = WORKSPACE_WIDTH / 2.0
        ws_cy = WORKSPACE_HEIGHT / 2.0
        rx, ry = _rotate_point(cx_ws, cy_ws, ws_cx, ws_cy, self._map_rotation)
        center_cx = view_cx + (rx - ws_cx) * scale
        center_cy = view_cy + (ry - ws_cy) * scale
        rel = [(x - cx_ws, y - cy_ws) for x, y in corners_ws]
        rel_rot = [_rotate_point(rx, ry, 0, 0, self._map_rotation) for rx, ry in rel]
        corners = [(center_cx + rx * scale, center_cy + ry * scale) for rx, ry in rel_rot]
        self._draft_item = self._canvas.create_polygon(
            [coord for point in corners for coord in point],
            fill='#f59e0b',
            stipple='gray50',
            outline='#ffffff',
            width=2,
            dash=(4, 2),
        )

    def _create_annotation_from_drag(self, start: tuple[float, float], end: tuple[float, float]):
        x1, y1 = start
        x2, y2 = end
        width = abs(x2 - x1)
        height = abs(y2 - y1)
        if width < 12.0 or height < 12.0:
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
            if _point_in_polygon(*self._annotation_canvas_center(annotation), polygon)
        }
        self._redraw()
        self._update_status('lasso complete')
        self._log(f'Lasso selected {len(self._selected_ids)} annotation(s)')

    def _clear_draft_items(self):
        if self._draft_item is not None:
            self._canvas.delete(self._draft_item)
            self._draft_item = None

    def _clear_interaction(self):
        self._drag_start_map = None
        self._last_drag_map = None
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
