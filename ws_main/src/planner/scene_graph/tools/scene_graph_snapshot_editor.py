#!/usr/bin/env python3
"""Standalone snapshot editor for SceneGraph saved_data snapshots.

Features:
- Visualize area boxes and object point clouds with Open3D GUI
- Edit area room label / description
- Edit object label
- Delete objects and clean related JSON references
- Save in-place or save as another relative snapshot folder
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import shutil
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import open3d as o3d
from open3d.visualization import gui, rendering


def create_render_material(shader: str = "defaultUnlit", point_size: int = 3, line_width: float = 1.0):
    if hasattr(rendering, "MaterialRecord"):
        material = rendering.MaterialRecord()
    else:
        material = rendering.Material()
    material.shader = shader
    if hasattr(material, "point_size"):
        material.point_size = point_size
    if hasattr(material, "line_width"):
        material.line_width = line_width
    return material


def now_readable_string() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def now_compact_string() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def sanitize_path_segment(raw_segment: str) -> str:
    chars: List[str] = []
    for ch in raw_segment:
        if ch.isalnum() or ch in {"_", "-"}:
            chars.append(ch)
        else:
            chars.append("_")
    sanitized = "".join(chars).strip("_")
    return sanitized


def normalize_relative_snapshot_folder(raw_path: str, allow_empty_default: bool = False) -> Optional[str]:
    if not raw_path:
        return f"snapshot_{now_compact_string()}" if allow_empty_default else None

    raw = raw_path.strip()
    if not raw:
        return f"snapshot_{now_compact_string()}" if allow_empty_default else None
    if raw.startswith("/"):
        return None

    parts: List[str] = []
    for token in raw.split("/"):
        if token in {"", "."}:
            continue
        if token == "..":
            return None
        sanitized = sanitize_path_segment(token) or "segment"
        parts.append(sanitized)

    if not parts:
        return f"snapshot_{now_compact_string()}" if allow_empty_default else None
    return "/".join(parts)


def color3(values: Sequence[float], fallback: Sequence[float]) -> List[float]:
    out: List[float] = []
    for i in range(3):
        val = values[i] if i < len(values) else fallback[i]
        try:
            fval = float(val)
        except (TypeError, ValueError):
            fval = float(fallback[i])
        if not math.isfinite(fval):
            fval = float(fallback[i])
        out.append(max(0.0, min(1.0, fval)))
    return out


def vec3(values: Sequence[float], fallback: Sequence[float]) -> List[float]:
    out: List[float] = []
    for i in range(3):
        val = values[i] if i < len(values) else fallback[i]
        try:
            fval = float(val)
        except (TypeError, ValueError):
            fval = float(fallback[i])
        if not math.isfinite(fval):
            fval = float(fallback[i])
        out.append(fval)
    return out


def pointcloud_has_points(cloud: Optional[o3d.geometry.PointCloud]) -> bool:
    return cloud is not None and len(cloud.points) > 0


def clone_point_cloud(cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    return copy.deepcopy(cloud)


def make_area_box(box_min: Sequence[float], box_max: Sequence[float], color: Sequence[float]) -> o3d.geometry.LineSet:
    aabb = o3d.geometry.AxisAlignedBoundingBox(box_min, box_max)
    box = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(aabb)
    colors = [list(color)] * len(box.lines)
    box.colors = o3d.utility.Vector3dVector(colors)
    return box


def make_line_set(
    points: Sequence[Sequence[float]], lines: Sequence[Sequence[int]], colors: Sequence[Sequence[float]]
) -> o3d.geometry.LineSet:
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector([list(point) for point in points])
    line_set.lines = o3d.utility.Vector2iVector([list(line) for line in lines])
    line_set.colors = o3d.utility.Vector3dVector([list(color) for color in colors])
    return line_set


def make_sphere(center: Sequence[float], radius: float, color: Sequence[float]) -> o3d.geometry.TriangleMesh:
    mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=16)
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color(list(color))
    mesh.translate(center)
    return mesh


def make_bbox_from_points(points: Iterable[Sequence[float]]) -> Optional[o3d.geometry.AxisAlignedBoundingBox]:
    pts = [list(p) for p in points]
    if not pts:
        return None
    return o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(pts))


class AreaItem:
    def __init__(self, area_id: int, data: dict):
        self.area_id = area_id
        self.data = data

    @property
    def room_label(self) -> str:
        return str(self.data.get("room_label", ""))

    @property
    def room_description(self) -> str:
        return str(self.data.get("room_description", ""))

    @property
    def center(self) -> List[float]:
        return vec3(self.data.get("center", [0.0, 0.0, 0.0]), [0.0, 0.0, 0.0])

    @property
    def box_min(self) -> List[float]:
        return vec3(self.data.get("box_min", [-0.1, -0.1, -0.1]), [-0.1, -0.1, -0.1])

    @property
    def box_max(self) -> List[float]:
        return vec3(self.data.get("box_max", [0.1, 0.1, 0.1]), [0.1, 0.1, 0.1])

    @property
    def color(self) -> List[float]:
        return color3(self.data.get("color", [0.3, 0.7, 0.9]), [0.3, 0.7, 0.9])

    @property
    def neighbor_area_ids(self) -> List[int]:
        neighbor_ids = self.data.get("neighbor_area_ids", [])
        out: List[int] = []
        for neighbor_id in neighbor_ids:
            try:
                out.append(int(neighbor_id))
            except (TypeError, ValueError):
                continue
        return out

    def display_name(self) -> str:
        label = self.room_label or "Unknown"
        return f"Area {self.area_id}: {label}"


class ObjectItem:
    def __init__(self, object_id: int, data: dict):
        self.object_id = object_id
        self.data = data

    @property
    def label(self) -> str:
        return str(self.data.get("label", "None"))

    @property
    def position(self) -> List[float]:
        return vec3(self.data.get("pos", [0.0, 0.0, 0.0]), [0.0, 0.0, 0.0])

    @property
    def color(self) -> List[float]:
        return color3(self.data.get("color", [0.9, 0.4, 0.1]), [0.9, 0.4, 0.1])

    @property
    def cloud_rel_path(self) -> str:
        return str(self.data.get("files", {}).get("cloud", ""))

    def display_name(self) -> str:
        return f"Obj {self.object_id}: {self.label}"


class SnapshotStore:
    def __init__(self, package_dir: Path, snapshot_dir: Path):
        self.package_dir = package_dir
        self.saved_data_dir = self.package_dir / "saved_data"
        self.snapshot_dir = snapshot_dir
        self.scene_graph_path = self.snapshot_dir / "scene_graph.json"
        self.manifest_path = self.snapshot_dir / "manifest.json"

        with self.scene_graph_path.open("r", encoding="utf-8") as f:
            self.root = json.load(f)
        with self.manifest_path.open("r", encoding="utf-8") as f:
            self.manifest = json.load(f)

        self._rebuild_indices()
        self.cloud_cache: Dict[int, Optional[o3d.geometry.PointCloud]] = {}

    def _rebuild_indices(self) -> None:
        self.areas_by_id: Dict[int, dict] = {}
        for area in self.root.get("areas", []):
            self.areas_by_id[int(area["id"])] = area

        self.objects_by_id: Dict[int, dict] = {}
        for obj in self.root.get("objects", []):
            self.objects_by_id[int(obj["id"])] = obj

    def area_items(self) -> List[AreaItem]:
        return [AreaItem(area_id, self.areas_by_id[area_id]) for area_id in sorted(self.areas_by_id)]

    def object_items(self) -> List[ObjectItem]:
        return [ObjectItem(obj_id, self.objects_by_id[obj_id]) for obj_id in sorted(self.objects_by_id)]

    def get_area(self, area_id: int) -> Optional[AreaItem]:
        if area_id not in self.areas_by_id:
            return None
        return AreaItem(area_id, self.areas_by_id[area_id])

    def get_object(self, object_id: int) -> Optional[ObjectItem]:
        if object_id not in self.objects_by_id:
            return None
        return ObjectItem(object_id, self.objects_by_id[object_id])

    def update_area(self, area_id: int, room_label: str, room_description: str) -> None:
        area = self.areas_by_id[area_id]
        area["room_label"] = room_label.strip()
        area["room_description"] = room_description.strip()

    def update_object_label(self, object_id: int, label: str) -> None:
        self.objects_by_id[object_id]["label"] = label.strip()

    def delete_object(self, object_id: int) -> None:
        if object_id not in self.objects_by_id:
            return

        self.root["objects"] = [obj for obj in self.root.get("objects", []) if int(obj.get("id", -1)) != object_id]
        for poly in self.root.get("polyhedrons", []):
            poly["object_ids"] = [oid for oid in poly.get("object_ids", []) if int(oid) != object_id]
        for area in self.root.get("areas", []):
            area["object_ids"] = [oid for oid in area.get("object_ids", []) if int(oid) != object_id]
        for obj in self.root.get("objects", []):
            edge = obj.get("edge", {})
            if int(edge.get("father_object_id", -1)) == object_id:
                edge["father_object_id"] = -1
            edge["child_object_ids"] = [cid for cid in edge.get("child_object_ids", []) if int(cid) != object_id]

        self.cloud_cache.pop(object_id, None)
        self._rebuild_indices()

    def _update_summary_fields(self, save_name: str) -> None:
        self.root["save_name"] = save_name
        self.root["saved_at"] = now_readable_string()
        counters = self.root.setdefault("counters", {})
        counters["area_count"] = len(self.root.get("areas", []))
        counters["object_count"] = len(self.root.get("objects", []))

        self.manifest["format_version"] = self.manifest.get("format_version", 1)
        self.manifest["save_name"] = save_name
        self.manifest["saved_at"] = self.root["saved_at"]
        self.manifest["scene_graph_file"] = "scene_graph.json"
        self.manifest["object_dir"] = "objects"
        summary = self.manifest.setdefault("summary", {})
        summary["poly_count"] = len(self.root.get("polyhedrons", []))
        summary["area_count"] = len(self.root.get("areas", []))
        summary["object_count"] = len(self.root.get("objects", []))
        summary["saved_cloud_num"] = sum(1 for obj in self.root.get("objects", []) if obj.get("files", {}).get("cloud"))

    def validate(self) -> None:
        object_ids = {int(obj["id"]) for obj in self.root.get("objects", [])}
        if len(object_ids) != len(self.root.get("objects", [])):
            raise ValueError("Duplicate object ids detected.")

        for poly in self.root.get("polyhedrons", []):
            for object_id in poly.get("object_ids", []):
                if int(object_id) not in object_ids:
                    raise ValueError(f"Dangling polyhedron.object_ids reference: {object_id}")
        for area in self.root.get("areas", []):
            for object_id in area.get("object_ids", []):
                if int(object_id) not in object_ids:
                    raise ValueError(f"Dangling area.object_ids reference: {object_id}")
        for obj in self.root.get("objects", []):
            edge = obj.get("edge", {})
            father_object_id = int(edge.get("father_object_id", -1))
            if father_object_id != -1 and father_object_id not in object_ids:
                raise ValueError(f"Dangling object.edge.father_object_id reference: {father_object_id}")
            for child_id in edge.get("child_object_ids", []):
                if int(child_id) not in object_ids:
                    raise ValueError(f"Dangling object.edge.child_object_ids reference: {child_id}")

    def _iter_object_files(self, obj: dict) -> Iterable[Tuple[str, Path]]:
        files = obj.get("files", {})
        for key in ("cloud", "obb_corners", "obb_axis"):
            rel = str(files.get(key, "") or "")
            if not rel:
                continue
            yield rel, self.snapshot_dir / rel

    def save_to(self, target_snapshot_dir: Path, relative_save_name: str) -> None:
        self.validate()
        self._update_summary_fields(relative_save_name)

        target_objects_dir = target_snapshot_dir / "objects"
        target_objects_dir.mkdir(parents=True, exist_ok=True)

        keep_rel_paths: Set[str] = set()
        for obj in self.root.get("objects", []):
            for rel_path, src_path in self._iter_object_files(obj):
                keep_rel_paths.add(rel_path)
                dst_path = target_snapshot_dir / rel_path
                dst_path.parent.mkdir(parents=True, exist_ok=True)
                if src_path.exists() and src_path.resolve() != dst_path.resolve():
                    shutil.copy2(src_path, dst_path)

        if target_snapshot_dir.resolve() == self.snapshot_dir.resolve():
            for existing in target_objects_dir.glob("*.pcd"):
                rel = existing.relative_to(target_snapshot_dir).as_posix()
                if rel not in keep_rel_paths:
                    existing.unlink()
        else:
            for existing in target_objects_dir.glob("*.pcd"):
                rel = existing.relative_to(target_snapshot_dir).as_posix()
                if rel not in keep_rel_paths:
                    existing.unlink()

        with (target_snapshot_dir / "scene_graph.json").open("w", encoding="utf-8") as f:
            json.dump(self.root, f, indent=2)
            f.write("\n")
        with (target_snapshot_dir / "manifest.json").open("w", encoding="utf-8") as f:
            json.dump(self.manifest, f, indent=2)
            f.write("\n")

    def overwrite(self) -> None:
        relative_name = self.snapshot_dir.relative_to(self.saved_data_dir).as_posix()
        self.save_to(self.snapshot_dir, relative_name)

    def save_as(self, relative_folder: str) -> Path:
        normalized = normalize_relative_snapshot_folder(relative_folder, allow_empty_default=False)
        if normalized is None:
            raise ValueError("map_folder must be a valid relative path under saved_data.")
        target_snapshot_dir = self.saved_data_dir / normalized
        target_snapshot_dir.mkdir(parents=True, exist_ok=True)
        self.save_to(target_snapshot_dir, normalized)
        return target_snapshot_dir

    def load_object_cloud(self, object_id: int) -> Optional[o3d.geometry.PointCloud]:
        if object_id in self.cloud_cache:
            return self.cloud_cache[object_id]
        obj = self.objects_by_id.get(object_id)
        if obj is None:
            self.cloud_cache[object_id] = None
            return None

        rel_path = str(obj.get("files", {}).get("cloud", "") or "")
        if not rel_path:
            self.cloud_cache[object_id] = None
            return None

        cloud_path = self.snapshot_dir / rel_path
        if not cloud_path.exists():
            self.cloud_cache[object_id] = None
            return None

        cloud = o3d.io.read_point_cloud(str(cloud_path))
        if not pointcloud_has_points(cloud):
            self.cloud_cache[object_id] = None
            return None
        self.cloud_cache[object_id] = cloud
        return cloud

    def full_bounds(self) -> o3d.geometry.AxisAlignedBoundingBox:
        points: List[Sequence[float]] = []
        for area in self.area_items():
            points.append(area.box_min)
            points.append(area.box_max)
            points.append(area.center)
        for obj in self.object_items():
            points.append(obj.position)
            cloud = self.load_object_cloud(obj.object_id)
            if pointcloud_has_points(cloud):
                bounds = cloud.get_axis_aligned_bounding_box()
                points.append(bounds.min_bound)
                points.append(bounds.max_bound)

        if not points:
            points = [[-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]]
        bbox = make_bbox_from_points(points)
        assert bbox is not None
        return bbox

    def focus_bounds_for_area(self, area_id: int) -> o3d.geometry.AxisAlignedBoundingBox:
        area = self.get_area(area_id)
        if area is None:
            return self.full_bounds()
        box_min = area.box_min
        box_max = area.box_max
        padding = 0.5
        return o3d.geometry.AxisAlignedBoundingBox(
            [box_min[0] - padding, box_min[1] - padding, box_min[2] - padding],
            [box_max[0] + padding, box_max[1] + padding, box_max[2] + padding],
        )

    def focus_bounds_for_object(self, object_id: int) -> o3d.geometry.AxisAlignedBoundingBox:
        obj = self.get_object(object_id)
        if obj is None:
            return self.full_bounds()
        cloud = self.load_object_cloud(object_id)
        if pointcloud_has_points(cloud):
            bbox = cloud.get_axis_aligned_bounding_box()
            bbox = bbox.scale(1.5, bbox.get_center())
            return bbox
        pos = obj.position
        padding = 0.5
        return o3d.geometry.AxisAlignedBoundingBox(
            [pos[0] - padding, pos[1] - padding, pos[2] - padding],
            [pos[0] + padding, pos[1] + padding, pos[2] + padding],
        )


class SnapshotEditorApp:
    def __init__(self, store: SnapshotStore):
        self.store = store
        self.geometry_names: Set[str] = set()
        self.scene_label_handles: List[object] = []
        self.selected_area_id: Optional[int] = None
        self.selected_object_id: Optional[int] = None
        self.focused_object_id: Optional[int] = None

        app = gui.Application.instance
        app.initialize()

        self.window = app.create_window("Scene Graph Snapshot Editor", 1680, 960)
        self.window.set_on_layout(self._on_layout)

        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        self.scene.scene.set_background([0.98, 0.98, 0.98, 1.0])

        self.panel = gui.Vert(0, gui.Margins(8, 8, 8, 8))
        self._build_panel()

        self.window.add_child(self.scene)
        self.window.add_child(self.panel)

        self._refresh_selectors()
        self._refresh_scene(reset_camera=True)

    def _build_panel(self) -> None:
        em = self.window.theme.font_size
        self.panel.add_child(gui.Label(f"Snapshot: {self.store.snapshot_dir.relative_to(self.store.saved_data_dir).as_posix()}"))
        self.status_label = gui.Label("Ready.")
        self.panel.add_child(self.status_label)

        self.panel.add_fixed(em * 0.5)
        self.panel.add_child(gui.Label("Area"))
        self.area_combo = gui.Combobox()
        self.area_combo.set_on_selection_changed(self._on_area_selected)
        self.panel.add_child(self.area_combo)
        self.area_type_edit = gui.TextEdit()
        self.area_desc_edit = gui.TextEdit()
        self.panel.add_child(gui.Label("Room Label"))
        self.panel.add_child(self.area_type_edit)
        self.panel.add_child(gui.Label("Room Description"))
        self.panel.add_child(self.area_desc_edit)
        area_buttons = gui.Horiz()
        self.area_apply_btn = gui.Button("Apply Area")
        self.area_apply_btn.set_on_clicked(self._apply_area_changes)
        self.area_focus_btn = gui.Button("Focus Area")
        self.area_focus_btn.set_on_clicked(self._focus_selected_area)
        area_buttons.add_child(self.area_apply_btn)
        area_buttons.add_child(self.area_focus_btn)
        self.panel.add_child(area_buttons)

        self.panel.add_fixed(em)
        self.panel.add_child(gui.Label("Object"))
        self.object_combo = gui.Combobox()
        self.object_combo.set_on_selection_changed(self._on_object_selected)
        self.panel.add_child(self.object_combo)
        self.object_label_edit = gui.TextEdit()
        self.panel.add_child(gui.Label("Object Label"))
        self.panel.add_child(self.object_label_edit)
        object_buttons = gui.Horiz()
        self.object_apply_btn = gui.Button("Apply Label")
        self.object_apply_btn.set_on_clicked(self._apply_object_changes)
        self.object_focus_btn = gui.Button("Focus Object")
        self.object_focus_btn.set_on_clicked(self._focus_selected_object)
        object_buttons.add_child(self.object_apply_btn)
        object_buttons.add_child(self.object_focus_btn)
        self.panel.add_child(object_buttons)
        self.object_delete_btn = gui.Button("Delete Object")
        self.object_delete_btn.set_on_clicked(self._delete_selected_object)
        self.panel.add_child(self.object_delete_btn)

        self.panel.add_fixed(em)
        self.panel.add_child(gui.Label("Save"))
        save_buttons = gui.Horiz()
        self.save_btn = gui.Button("Overwrite")
        self.save_btn.set_on_clicked(self._overwrite_snapshot)
        self.refocus_btn = gui.Button("Reset View")
        self.refocus_btn.set_on_clicked(self._reset_camera)
        save_buttons.add_child(self.save_btn)
        save_buttons.add_child(self.refocus_btn)
        self.panel.add_child(save_buttons)

        self.panel.add_child(gui.Label("Save As Relative Folder"))
        self.save_as_edit = gui.TextEdit()
        self.save_as_edit.text_value = ""
        self.panel.add_child(self.save_as_edit)
        self.save_as_btn = gui.Button("Save As")
        self.save_as_btn.set_on_clicked(self._save_as_snapshot)
        self.panel.add_child(self.save_as_btn)

    def _on_layout(self, _layout_context: gui.LayoutContext) -> None:
        rect = self.window.content_rect
        panel_width = 380
        self.scene.frame = gui.Rect(rect.x, rect.y, rect.width - panel_width, rect.height)
        self.panel.frame = gui.Rect(self.scene.frame.get_right(), rect.y, panel_width, rect.height)

    def _set_status(self, text: str) -> None:
        self.status_label.text = text

    def _show_dialog(self, title: str, message: str) -> None:
        dialog = gui.Dialog(title)
        em = self.window.theme.font_size
        layout = gui.Vert(0, gui.Margins(em, em, em, em))
        layout.add_child(gui.Label(message))
        ok = gui.Button("OK")
        ok.set_on_clicked(lambda: self.window.close_dialog())
        layout.add_fixed(em * 0.5)
        layout.add_child(ok)
        dialog.add_child(layout)
        self.window.show_dialog(dialog)

    def _refresh_selectors(self) -> None:
        self._reload_area_combo()
        self._reload_object_combo()

    def _reload_area_combo(self) -> None:
        items = self.store.area_items()
        self.area_combo.clear_items()
        for area in items:
            self.area_combo.add_item(area.display_name())

        if items:
            if self.selected_area_id not in {item.area_id for item in items}:
                self.selected_area_id = items[0].area_id
            idx = next(i for i, item in enumerate(items) if item.area_id == self.selected_area_id)
            self.area_combo.selected_index = idx
            self._populate_area_fields(self.selected_area_id)
        else:
            self.selected_area_id = None
            self.area_type_edit.text_value = ""
            self.area_desc_edit.text_value = ""

    def _reload_object_combo(self) -> None:
        items = self.store.object_items()
        self.object_combo.clear_items()
        for obj in items:
            self.object_combo.add_item(obj.display_name())

        if items:
            if self.selected_object_id not in {item.object_id for item in items}:
                self.selected_object_id = items[0].object_id
            idx = next(i for i, item in enumerate(items) if item.object_id == self.selected_object_id)
            self.object_combo.selected_index = idx
            self._populate_object_fields(self.selected_object_id)
        else:
            self.selected_object_id = None
            self.object_label_edit.text_value = ""

    def _populate_area_fields(self, area_id: Optional[int]) -> None:
        if area_id is None:
            self.area_type_edit.text_value = ""
            self.area_desc_edit.text_value = ""
            return
        area = self.store.get_area(area_id)
        if area is None:
            return
        self.area_type_edit.text_value = area.room_label
        self.area_desc_edit.text_value = area.room_description

    def _populate_object_fields(self, object_id: Optional[int]) -> None:
        if object_id is None:
            self.object_label_edit.text_value = ""
            return
        obj = self.store.get_object(object_id)
        if obj is None:
            return
        self.object_label_edit.text_value = obj.label

    def _on_area_selected(self, _text: str, index: int) -> None:
        items = self.store.area_items()
        if 0 <= index < len(items):
            self.selected_area_id = items[index].area_id
            self.focused_object_id = None
            self._populate_area_fields(self.selected_area_id)
            self._refresh_scene(reset_camera=False)

    def _on_object_selected(self, _text: str, index: int) -> None:
        items = self.store.object_items()
        if 0 <= index < len(items):
            self.selected_object_id = items[index].object_id
            if self.focused_object_id != self.selected_object_id:
                self.focused_object_id = None
            self._populate_object_fields(self.selected_object_id)
            self._refresh_scene(reset_camera=False)

    def _clear_scene(self) -> None:
        for name in list(self.geometry_names):
            self.scene.scene.remove_geometry(name)
        self.geometry_names.clear()
        for label in self.scene_label_handles:
            self.scene.remove_3d_label(label)
        self.scene_label_handles.clear()

    def _add_geometry(self, name: str, geometry: o3d.geometry.Geometry, shader: str = "defaultUnlit", point_size: int = 3) -> None:
        line_width = 2.0 if shader == "unlitLine" else 1.0
        material = create_render_material(shader=shader, point_size=point_size, line_width=line_width)
        self.scene.scene.add_geometry(name, geometry, material)
        self.geometry_names.add(name)

    def _add_area_connection_geometries(self, areas: Sequence[AreaItem]) -> None:
        area_centers = {area.area_id: area.center for area in areas}
        line_points: List[List[float]] = []
        line_indices: List[List[int]] = []
        line_colors: List[List[float]] = []
        drawn_edges: Set[Tuple[int, int]] = set()

        for area in areas:
            for neighbor_id in area.neighbor_area_ids:
                if neighbor_id not in area_centers or neighbor_id == area.area_id:
                    continue
                edge_key = tuple(sorted((area.area_id, neighbor_id)))
                if edge_key in drawn_edges:
                    continue
                drawn_edges.add(edge_key)

                point_index = len(line_points)
                line_points.append(list(area_centers[area.area_id]))
                line_points.append(list(area_centers[neighbor_id]))
                line_indices.append([point_index, point_index + 1])

                is_selected_edge = self.selected_area_id in edge_key
                line_colors.append([0.95, 0.45, 0.15] if is_selected_edge else [0.7, 0.7, 0.7])

        if not line_points:
            return

        line_set = make_line_set(line_points, line_indices, line_colors)
        self._add_geometry("area_connections", line_set, shader="unlitLine")

    def _add_area_geometries(self, areas: Sequence[AreaItem]) -> None:
        for area in areas:
            area_color = area.color
            if area.area_id == self.selected_area_id:
                area_color = [1.0, 0.4, 0.1]
            area_box = make_area_box(area.box_min, area.box_max, area_color)
            self._add_geometry(f"area_box_{area.area_id}", area_box, shader="unlitLine")

            area_center_color = area.color if area.area_id != self.selected_area_id else [1.0, 0.5, 0.1]
            area_center_radius = 0.08 if area.area_id != self.selected_area_id else 0.12
            area_center = make_sphere(area.center, area_center_radius, area_center_color)
            self._add_geometry(f"area_center_{area.area_id}", area_center, shader="defaultLit")

            area_label = area.room_label or "Unknown"
            label_handle = self.scene.add_3d_label(area.center, f"Area {area.area_id}: {area_label}")
            self.scene_label_handles.append(label_handle)

    def _add_object_geometries(self, objects: Sequence[ObjectItem]) -> None:
        focused_object_id = self.focused_object_id
        dim_other_objects = focused_object_id is not None

        for obj in objects:
            cloud = self.store.load_object_cloud(obj.object_id)
            object_is_focused = obj.object_id == focused_object_id

            if pointcloud_has_points(cloud):
                render_cloud = clone_point_cloud(cloud)
                if dim_other_objects and not object_is_focused:
                    render_cloud.paint_uniform_color([0.65, 0.65, 0.65])
                elif object_is_focused:
                    render_cloud.paint_uniform_color([0.0, 1.0, 0.0])
                self._add_geometry(f"object_cloud_{obj.object_id}", render_cloud, point_size=4)

            label_position = list(obj.position)
            label_position[2] += 0.12
            label_handle = self.scene.add_3d_label(label_position, f"[{obj.object_id}]")
            self.scene_label_handles.append(label_handle)

    def _refresh_scene(self, reset_camera: bool = False) -> None:
        self._clear_scene()
        areas = self.store.area_items()
        self._add_area_connection_geometries(areas)
        self._add_area_geometries(areas)
        self._add_object_geometries(self.store.object_items())

        if reset_camera:
            self._set_camera_to_bounds(self.store.full_bounds())

    def _set_camera_to_bounds(self, bounds: o3d.geometry.AxisAlignedBoundingBox) -> None:
        center = bounds.get_center()
        self.scene.setup_camera(60.0, bounds, center)

    def _reset_camera(self) -> None:
        self.focused_object_id = None
        self._refresh_scene(reset_camera=False)
        self._set_camera_to_bounds(self.store.full_bounds())

    def _focus_selected_area(self) -> None:
        if self.selected_area_id is None:
            return
        self.focused_object_id = None
        self._refresh_scene(reset_camera=False)
        self._set_camera_to_bounds(self.store.focus_bounds_for_area(self.selected_area_id))

    def _focus_selected_object(self) -> None:
        if self.selected_object_id is None:
            return
        self.focused_object_id = self.selected_object_id
        self._refresh_scene(reset_camera=False)
        self._set_camera_to_bounds(self.store.focus_bounds_for_object(self.selected_object_id))

    def _apply_area_changes(self) -> None:
        if self.selected_area_id is None:
            self._show_dialog("No Area", "Select an area first.")
            return
        self.store.update_area(
            self.selected_area_id,
            self.area_type_edit.text_value,
            self.area_desc_edit.text_value,
        )
        self._reload_area_combo()
        self._set_status(f"Updated area {self.selected_area_id}.")

    def _apply_object_changes(self) -> None:
        if self.selected_object_id is None:
            self._show_dialog("No Object", "Select an object first.")
            return
        self.store.update_object_label(self.selected_object_id, self.object_label_edit.text_value)
        self._reload_object_combo()
        self._set_status(f"Updated object {self.selected_object_id}.")

    def _delete_selected_object(self) -> None:
        if self.selected_object_id is None:
            self._show_dialog("No Object", "Select an object first.")
            return
        deleted_id = self.selected_object_id
        self.store.delete_object(deleted_id)
        self.selected_object_id = None
        self._refresh_selectors()
        self._refresh_scene(reset_camera=False)
        self._set_status(f"Deleted object {deleted_id}.")

    def _overwrite_snapshot(self) -> None:
        try:
            self.store.overwrite()
        except Exception as exc:  # pragma: no cover - GUI error path
            self._show_dialog("Save Failed", str(exc))
            return
        self._set_status("Snapshot overwritten.")

    def _save_as_snapshot(self) -> None:
        target = self.save_as_edit.text_value.strip()
        try:
            saved_dir = self.store.save_as(target)
        except Exception as exc:  # pragma: no cover - GUI error path
            self._show_dialog("Save As Failed", str(exc))
            return
        self._set_status(f"Saved to {saved_dir.relative_to(self.store.saved_data_dir).as_posix()}.")

    def run(self) -> None:
        gui.Application.instance.run()


def resolve_default_snapshot_dir(saved_data_dir: Path) -> Optional[Path]:
    candidates = [path for path in saved_data_dir.iterdir() if path.is_dir()]
    if not candidates:
        return None
    candidates.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    return candidates[0]


def resolve_snapshot_dir(package_dir: Path, snapshot_arg: Optional[str]) -> Path:
    saved_data_dir = package_dir / "saved_data"
    if snapshot_arg:
        path = Path(snapshot_arg)
        if path.is_absolute():
            snapshot_dir = path
        else:
            normalized = normalize_relative_snapshot_folder(snapshot_arg, allow_empty_default=False)
            if normalized is None:
                raise ValueError("Snapshot argument must be a valid relative map_folder or an absolute snapshot path.")
            snapshot_dir = saved_data_dir / normalized
    else:
        snapshot_dir = resolve_default_snapshot_dir(saved_data_dir)
        if snapshot_dir is None:
            raise ValueError("No snapshot directory found under saved_data.")

    if not snapshot_dir.exists():
        raise FileNotFoundError(f"Snapshot directory not found: {snapshot_dir}")
    if not (snapshot_dir / "scene_graph.json").exists():
        raise FileNotFoundError(f"scene_graph.json not found in snapshot directory: {snapshot_dir}")
    if not (snapshot_dir / "manifest.json").exists():
        raise FileNotFoundError(f"manifest.json not found in snapshot directory: {snapshot_dir}")
    return snapshot_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SceneGraph snapshot editor with Open3D GUI")
    parser.add_argument(
        "snapshot",
        nargs="?",
        default=None,
        help="Relative map_folder under saved_data or absolute snapshot directory. Default: latest snapshot.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    script_path = Path(__file__).resolve()
    package_dir = script_path.parent.parent

    try:
        snapshot_dir = resolve_snapshot_dir(package_dir, args.snapshot)
        store = SnapshotStore(package_dir, snapshot_dir)
    except Exception as exc:
        print(f"[scene_graph_snapshot_editor] initialization failed: {exc}", file=sys.stderr)
        return 1

    app = SnapshotEditorApp(store)
    app.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
