# Scene Graph Snapshot Editor

Standalone Open3D GUI editor for `saved_data/<map_folder>` snapshots.

## Features

- Visualize area boxes and object point clouds
- Edit `areas[].room_label`
- Edit `areas[].room_description`
- Edit `objects[].label`
- Delete objects and clean related JSON references
- Overwrite current snapshot or save as another relative `map_folder`

## Usage

```bash
python3 src/planner/scene_graph/tools/scene_graph_snapshot_editor.py
python3 src/planner/scene_graph/tools/scene_graph_snapshot_editor.py snapshot_20260329_183344
python3 src/planner/scene_graph/tools/scene_graph_snapshot_editor.py exp1/run_a
```

If no argument is given, the editor opens the latest snapshot under `saved_data/`.

## Notes

- `Save As` only accepts relative folders under `saved_data`.
- Deleted objects are removed from:
  - `objects`
  - `polyhedrons[].object_ids`
  - `areas[].object_ids`
  - object parent/child references
- The tool writes `scene_graph.json` and `manifest.json` directly; keep backups for important datasets.
