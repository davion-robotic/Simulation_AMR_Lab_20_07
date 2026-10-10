#!/usr/bin/env python3
"""topology_edit.py: Pure Python Module chuyên trách chỉnh sửa và lưu trữ Topology Graph.

Bao gồm:
- Quy trình 4 bước chuẩn: Load -> Mutate -> Reindex -> Save (Atomic Write).
- Lớp bảo vệ 1: Validate tính hợp lệ của mảng vertices và lanes (chặn self-loop, kiểm tra kiểu dữ liệu).
- Lớp bảo vệ 2: Safety check khoảng cách vị trí Robot & trạng thái di chuyển (kiểm tra cả các làn nối).
- Thao tác theo Intent: add_vertex, delete_vertex, add_lane, delete_lane, full_save.
- Quản lý file riêng cho từng map (<map_name>_topology.yaml) và sync nav_graph.yaml.
"""

import math
import os
import threading
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from mybot_common.command_utils import sanitize_map_name

RMF_MAPS_DIR = Path(
    '/home/nam/ROBOT_AMR_LAB20_07/AMR_Simulation/AMR_Simulation/'
    'src/mybot_rmf/maps'
)


def project_point_on_segment(cx: float, cy: float, ax: float, ay: float, bx: float, by: float):
    """Tính điểm chiếu vuông góc của C(cx, cy) lên đoạn thẳng AB."""
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return (ax, ay), math.hypot(cx - ax, cy - ay)
    t = ((cx - ax) * dx + (cy - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    px = ax + t * dx
    py = ay + t * dy
    dist = math.hypot(cx - px, cy - py)
    return (px, py), dist


def resolve_vertex_index(val, vertices: list) -> Tuple[Optional[int], str]:
    """Helper tìm index của vertex từ int, string int, dict, hoặc name string (hỗ trợ case-insensitive & space-insensitive)."""
    if val is None:
        return None, ""

    # 1. Unwrap if dict
    if isinstance(val, dict):
        extracted = None
        for k in [
            'name', 'vertex_name', 'vertexName', 'vertex_id', 'vertexId', 'goal_id', 'goalId',
            'waypoint_name', 'waypointName', 'point_name', 'pointName', 'node_name', 'nodeName',
            'target_name', 'targetName', 'label', 'station_name', 'stationName', 'index', 'idx',
            'vertex_index', 'vertexIndex', 'id', 'data'
        ]:
            if k in val and val[k] is not None:
                extracted = val[k]
                break
        if extracted is None:
            if 'x' in val and 'y' in val:
                try:
                    fx, fy = float(val['x']), float(val['y'])
                    min_d = float('inf')
                    best_i = None
                    for i, v in enumerate(vertices):
                        d = math.hypot(float(v[0]) - fx, float(v[1]) - fy)
                        if d < min_d:
                            min_d = d
                            best_i = i
                    if best_i is not None and min_d < 0.8:
                        v_name = vertices[best_i][2].get('name', f'idx_{best_i}') if len(vertices[best_i]) > 2 and isinstance(vertices[best_i][2], dict) else f'idx_{best_i}'
                        return best_i, str(v_name)
                except Exception:
                    pass
            return None, ""
        val = extracted

    # 2. Unwrap if list or tuple
    if isinstance(val, (list, tuple)) and len(val) > 0:
        return resolve_vertex_index(val[0], vertices)

    num_v = len(vertices)

    # 3. Direct integer index check
    if isinstance(val, int) and not isinstance(val, bool):
        if 0 <= val < num_v:
            v_name = vertices[val][2].get('name', f'idx_{val}') if len(vertices[val]) > 2 and isinstance(vertices[val][2], dict) else f'idx_{val}'
            return val, str(v_name)
        return None, ""

    if isinstance(val, float) and val.is_integer():
        int_val = int(val)
        if 0 <= int_val < num_v:
            v_name = vertices[int_val][2].get('name', f'idx_{int_val}') if len(vertices[int_val]) > 2 and isinstance(vertices[int_val][2], dict) else f'idx_{int_val}'
            return int_val, str(v_name)

    val_str = str(val).strip()
    if not val_str:
        return None, ""

    val_clean = val_str.replace(' ', '').lower()
    for i, v in enumerate(vertices):
        if len(v) > 2 and isinstance(v[2], dict):
            v_name = str(v[2].get('name', '')).strip()
            if v_name.lower() == val_str.lower() or v_name.replace(' ', '').lower() == val_clean:
                return i, v_name

    if val_str.isdigit() or (val_str.startswith('-') and val_str[1:].isdigit()):
        idx = int(val_str)
        if 0 <= idx < num_v:
            v_name = vertices[idx][2].get('name', f'idx_{idx}') if len(vertices[idx]) > 2 and isinstance(vertices[idx][2], dict) else f'idx_{idx}'
            return idx, str(v_name)

    for i, v in enumerate(vertices):
        fallback_names = [f'idx_{i}', f'point_{i}', f'wp_{i}', f'p{i+1}', f'p_{i+1}']
        if val_clean in [fn.lower() for fn in fallback_names]:
            v_name = vertices[i][2].get('name', f'idx_{i}') if len(v) > 2 and isinstance(v[2], dict) else f'idx_{i}'
            return i, str(v_name)

    return None, ""


class TopologyEdit:
    """Lớp thuần Python chuyên trách xử lý chỉnh sửa Đồ thị Topology."""

    def __init__(self, maps_dir: Path = RMF_MAPS_DIR):
        self.maps_dir = maps_dir
        self._lock = threading.RLock()
        os.makedirs(self.maps_dir, exist_ok=True)

    def get_map_file_path(self, map_name: str, active_map: str = 'map_lab20') -> Path:
        safe_name = sanitize_map_name(map_name) or active_map
        return self.maps_dir / f'{safe_name}_topology.yaml'

    def load_graph(self, map_name: str = None, active_map: str = 'map_lab20') -> Tuple[dict, str, str, list, list]:
        target_map = sanitize_map_name(map_name) or active_map
        map_file_1 = self.maps_dir / f'{target_map}_topology.yaml'
        map_file_2 = self.maps_dir / f'{target_map}topology.yaml'
        map_file_3 = self.maps_dir / f'map_{target_map}_topology.yaml'

        file_to_read = None
        for candidate in [map_file_1, map_file_2, map_file_3]:
            if candidate.is_file():
                file_to_read = candidate
                break

        if file_to_read is None:
            empty_structure = {
                'levels': {'L1': {'vertices': [], 'lanes': []}},
                'lifts': {},
                'doors': {}
            }
            return empty_structure, target_map, 'L1', [], []

        try:
            with open(file_to_read, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}

            if not isinstance(data, dict):
                data = {}

            levels = data.get('levels', {})
            level_name = data.get('level_name') or data.get('level') or 'L1'

            if isinstance(levels, dict) and levels:
                level_name = next(iter(levels))
                level_data = levels.get(level_name, {})
                if isinstance(level_data, dict):
                    vertices = level_data.get('vertices', [])
                    lanes = level_data.get('lanes', [])
                else:
                    vertices, lanes = [], []
            elif 'vertices' in data or 'lanes' in data:
                vertices = data.get('vertices', [])
                lanes = data.get('lanes', [])
                data = {
                    **{k: v for k, v in data.items() if k not in ('vertices', 'lanes', 'level_name', 'level')},
                    'levels': {
                        level_name: {
                            'vertices': vertices,
                            'lanes': lanes,
                        }
                    }
                }
            else:
                level_name = 'L1'
                vertices, lanes = [], []
                data = {'levels': {'L1': {'vertices': [], 'lanes': []}}, **data}

            num_v = len(vertices)
            valid_lanes = []
            for lane in lanes:
                if isinstance(lane, (list, tuple)) and len(lane) >= 2:
                    try:
                        u, v = int(lane[0]), int(lane[1])
                        if 0 <= u < num_v and 0 <= v < num_v and u != v:
                            valid_lanes.append(lane)
                    except (ValueError, TypeError):
                        pass
            if isinstance(data.get('levels'), dict) and level_name in data['levels']:
                data['levels'][level_name]['lanes'] = valid_lanes
            lanes = valid_lanes

            return data, target_map, level_name, vertices, lanes
        except Exception:
            empty_structure = {'levels': {'L1': {'vertices': [], 'lanes': []}}, 'lifts': {}, 'doors': {}}
            return empty_structure, target_map, 'L1', [], []

    def validate(self, vertices: list, lanes: list) -> Tuple[bool, str]:
        """Validate tính toàn vẹn của vertices và lanes trong đồ thị Topology."""
        if not isinstance(vertices, list):
            return False, f'Vertices must be a list, got {type(vertices).__name__}'

        for i, v in enumerate(vertices):
            if not isinstance(v, (list, tuple)) or len(v) < 2:
                return False, f'Vertex at index {i} has invalid format (must have at least [x, y]): {v}'
            try:
                vx = float(v[0])
                vy = float(v[1])
                if math.isnan(vx) or math.isnan(vy) or math.isinf(vx) or math.isinf(vy):
                    return False, f'Vertex {i} coordinates are NaN or Infinity: ({vx}, {vy})'
            except (ValueError, TypeError):
                return False, f'Vertex {i} coordinates cannot be converted to float: ({v[0]}, {v[1]})'

        if not isinstance(lanes, list):
            return False, f'Lanes must be a list, got {type(lanes).__name__}'

        num_v = len(vertices)
        for i, lane in enumerate(lanes):
            if not isinstance(lane, (list, tuple)) or len(lane) < 2:
                return False, f'Lane at index {i} is invalid format: {lane}'
            try:
                u, v = int(lane[0]), int(lane[1])
            except (ValueError, TypeError):
                return False, f'Lane {i} indices cannot be converted to int: ({lane[0]}, {lane[1]})'

            if u < 0 or u >= num_v:
                return False, f'Lane {i} reference start index {u} out of bound [0, {num_v - 1}]'
            if v < 0 or v >= num_v:
                return False, f'Lane {i} reference end index {v} out of bound [0, {num_v - 1}]'
            if u == v:
                return False, f'Lane {i} is a self-loop ({u} -> {v}), which is not permitted'

        return True, 'Valid'

    def check_safety(
        self,
        vertices: list,
        lanes: list,
        robot_x: float,
        robot_y: float,
        robot_status: str,
        target_vertex_idx: int = None,
        target_lane: list = None,
        force: bool = False,
    ) -> Tuple[bool, str]:
        """Safety check: Chặn xóa waypoint / lane khi robot đang di chuyển gần đó."""
        if force:
            return True, 'Safety check bypassed with force=True'

        is_moving = str(robot_status).lower() in ['moving', 'navigating', 'preparing', 'executing']
        if not is_moving:
            return True, 'Safe: Robot is not moving'

        if target_vertex_idx is not None and 0 <= target_vertex_idx < len(vertices):
            v = vertices[target_vertex_idx]
            vx, vy = float(v[0]), float(v[1])
            v_name = v[2].get('name', f'idx_{target_vertex_idx}') if len(v) > 2 and isinstance(v[2], dict) else f'idx_{target_vertex_idx}'
            dist = math.hypot(robot_x - vx, robot_y - vy)
            if dist < 0.4:
                return False, f"Cannot delete vertex '{v_name}': Robot is actively moving near it (dist={dist:.2f}m)"

            for lane in lanes:
                l_u, l_v = int(lane[0]), int(lane[1])
                if l_u == target_vertex_idx or l_v == target_vertex_idx:
                    if l_u < len(vertices) and l_v < len(vertices):
                        ax, ay = float(vertices[l_u][0]), float(vertices[l_u][1])
                        bx, by = float(vertices[l_v][0]), float(vertices[l_v][1])
                        _, lane_dist = project_point_on_segment(robot_x, robot_y, ax, ay, bx, by)
                        if lane_dist < 0.4:
                            return False, f"Cannot delete vertex '{v_name}': Robot is actively moving along connected lane ({l_u}-{l_v}) (dist={lane_dist:.2f}m)"

        if target_lane is not None and len(target_lane) >= 2:
            u, v_idx = int(target_lane[0]), int(target_lane[1])
            if u < len(vertices) and v_idx < len(vertices):
                ax, ay = float(vertices[u][0]), float(vertices[u][1])
                bx, by = float(vertices[v_idx][0]), float(vertices[v_idx][1])
                _, dist = project_point_on_segment(robot_x, robot_y, ax, ay, bx, by)
                if dist < 0.4:
                    return False, f"Cannot delete lane ({u}-{v_idx}): Robot is actively moving near it (dist={dist:.2f}m)"

        return True, 'Safe'

    def save_atomic(self, yaml_content: dict, map_name: str = None, active_map: str = 'map_lab20') -> Tuple[bool, str]:
        with self._lock:
            target_map = sanitize_map_name(map_name) or active_map
            map_file_path = self.get_map_file_path(target_map, active_map)
            unique_id = uuid.uuid4().hex[:8]
            temp_map_path = self.maps_dir / f'{target_map}_topology_{unique_id}.tmp'

            try:
                with open(temp_map_path, 'w', encoding='utf-8') as f:
                    yaml.dump(yaml_content, f, default_flow_style=None, sort_keys=False)
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(temp_map_path, map_file_path)

                try:
                    from ament_index_python.packages import get_package_share_directory
                    share_dir = Path(get_package_share_directory('mybot_rmf')) / 'maps'
                    os.makedirs(share_dir, exist_ok=True)
                    share_file = share_dir / f'{target_map}_topology.yaml'
                    share_temp = share_dir / f'{target_map}_topology_{unique_id}.tmp'
                    with open(share_temp, 'w', encoding='utf-8') as f:
                        yaml.dump(yaml_content, f, default_flow_style=None, sort_keys=False)
                        f.flush()
                        os.fsync(f.fileno())
                    os.replace(share_temp, share_file)
                except Exception:
                    pass

                return True, str(map_file_path)

            except Exception as e:
                return False, str(e)
            finally:
                if temp_map_path.exists():
                    try:
                        os.remove(temp_map_path)
                    except Exception:
                        pass

    def add_vertex(self, payload: dict, active_map: str = 'map_lab20', is_edit: bool = False) -> dict:
        """Thêm hoặc cập nhật một điểm (vertex/waypoint) trong bản đồ Topology."""
        target_map = payload.get('map_name') or payload.get('mapName') or payload.get('map') or payload.get('active_map') or active_map

        v_sub = payload.get('vertex') or payload.get('point') or payload.get('waypoint') or payload.get('data')
        data_src = payload
        if isinstance(v_sub, dict):
            data_src = {**payload, **v_sub}

        old_name = str(data_src.get('old_name') or data_src.get('oldName') or '').strip()
        new_name = str(data_src.get('new_name') or data_src.get('newName') or data_src.get('rename_to') or data_src.get('renameTo') or data_src.get('target_name') or data_src.get('targetName') or '').strip()

        name = ''
        for k in [
            'name', 'vertex_name', 'vertexName', 'vertex_id', 'vertexId', 'goal_id', 'goalId',
            'waypoint_name', 'waypointName', 'point_name', 'pointName', 'node_name', 'nodeName',
            'target_name', 'targetName', 'station_name', 'stationName', 'label', 'id'
        ]:
            if k in data_src and data_src[k] is not None:
                val = str(data_src[k]).strip()
                if val:
                    name = val
                    break

        if new_name and not name:
            name = new_name

        target_idx = None
        for k in ['index', 'idx', 'vertex_index', 'vertexIndex', 'point_index', 'pointIndex', 'node_index', 'nodeIndex']:
            if k in data_src and data_src[k] is not None:
                val = data_src[k]
                if isinstance(val, int) and not isinstance(val, bool):
                    target_idx = val
                    break
                val_str = str(val).strip()
                if val_str.isdigit():
                    target_idx = int(val_str)
                    break

        full_data, map_name, level_name, vertices, lanes = self.load_graph(target_map, active_map)

        existing_idx = None
        if target_idx is not None and 0 <= target_idx < len(vertices):
            existing_idx = target_idx
        else:
            for search_name in (old_name, name):
                if not search_name:
                    continue
                search_clean = search_name.replace(' ', '').lower()
                for i, v in enumerate(vertices):
                    if len(v) > 2 and isinstance(v[2], dict):
                        v_name = str(v[2].get('name', '')).strip()
                        if v_name.lower() == search_name.lower() or v_name.replace(' ', '').lower() == search_clean:
                            existing_idx = i
                            break
                if existing_idx is not None:
                    break

        if not is_edit and not old_name and target_idx is None:
            if existing_idx is not None:
                existing_names = {
                    str(v[2].get('name', '')).strip().lower()
                    for v in vertices
                    if len(v) > 2 and isinstance(v[2], dict) and v[2].get('name')
                }
                cand_num = len(vertices) + 1
                while f'p{cand_num}' in existing_names:
                    cand_num += 1
                name = f'P{cand_num}'
                existing_idx = None

        if is_edit and existing_idx is None:
            target_query = old_name or name or target_idx
            raise KeyError(f"Vertex '{target_query}' not found to edit in map '{target_map}'")

        default_x = vertices[existing_idx][0] if existing_idx is not None else 0.0
        default_y = vertices[existing_idx][1] if existing_idx is not None else 0.0

        try:
            x = float(data_src['x']) if 'x' in data_src and data_src['x'] is not None else float(default_x)
            y = float(data_src['y']) if 'y' in data_src and data_src['y'] is not None else float(default_y)
        except (ValueError, TypeError) as e:
            raise ValueError(f'Invalid x or y coordinates: {e}')

        if not name:
            if existing_idx is not None and len(vertices[existing_idx]) > 2 and isinstance(vertices[existing_idx][2], dict):
                name = str(vertices[existing_idx][2].get('name') or f'P{existing_idx + 1}').strip()
            else:
                existing_names = {
                    str(v[2].get('name', '')).strip().lower()
                    for v in vertices
                    if len(v) > 2 and isinstance(v[2], dict) and v[2].get('name')
                }
                cand_num = len(vertices) + 1
                while f'p{cand_num}' in existing_names:
                    cand_num += 1
                name = f'P{cand_num}'

        final_name = new_name if new_name else name

        props = {}
        if existing_idx is not None and len(vertices[existing_idx]) > 2 and isinstance(vertices[existing_idx][2], dict):
            props = vertices[existing_idx][2].copy()

        props.update({
            'name': final_name,
            'is_parking_spot': bool(data_src.get('is_parking_spot', props.get('is_parking_spot', False))),
            'is_charger': bool(data_src.get('is_charger', props.get('is_charger', False))),
            'is_pickup': bool(data_src.get('is_pickup', props.get('is_pickup', False))),
            'is_destination': bool(data_src.get('is_destination', props.get('is_destination', False))),
            'is_boost': bool(data_src.get('is_boost', props.get('is_boost', False))),
            'yaw': float(data_src.get('yaw', props.get('yaw', 0.0)))
        })

        if existing_idx is not None:
            vertices[existing_idx] = [x, y, props]
            msg = f"Updated existing vertex '{final_name}' at index {existing_idx} for map '{target_map}'"
        else:
            vertices.append([x, y, props])
            msg = f"Added new vertex '{final_name}' for map '{target_map}' (total: {len(vertices)})"

        valid, err = self.validate(vertices, lanes)
        if not valid:
            raise ValueError(f'Validation failed after add_vertex: {err}')

        full_data['levels'][level_name]['vertices'] = vertices
        full_data['levels'][level_name]['lanes'] = lanes

        success, save_msg = self.save_atomic(full_data, map_name=target_map, active_map=active_map)
        if not success:
            raise IOError(f'Atomic save failed: {save_msg}')

        if existing_idx is not None and old_name and old_name != final_name:
            try:
                from mybot_goal.goal_storage import GoalStorage
                g_storage = GoalStorage()
                g_storage.delete_goal(target_map, old_name)
            except Exception:
                pass

        return {
            'status': 'ok',
            'action': 'edit_vertex' if existing_idx is not None else 'add_vertex',
            'map_name': target_map,
            'vertex_name': final_name,
            'message': msg,
            'vertices_count': len(vertices)
        }

    def edit_vertex(self, payload: dict, active_map: str = 'map_lab20') -> dict:
        """Chỉnh sửa thông tin đỉnh (tọa độ, tên, thuộc tính) trong bản đồ Topology."""
        return self.add_vertex(payload, active_map=active_map, is_edit=True)

    def edit_lane(self, payload: dict, active_map: str = 'map_lab20') -> dict:
        """Chỉnh sửa thuộc tính làn đường (speed limit, bidirectional) trong bản đồ Topology."""
        return self.add_lane(payload, active_map=active_map)

    def delete_vertex(
        self,
        payload: dict,
        active_map: str = 'map_lab20',
        robot_x: float = 0.0,
        robot_y: float = 0.0,
        robot_status: str = 'idle'
    ) -> dict:
        """Xóa một điểm (vertex) và các làn đường nối với nó, tự động reindex lại các lanes còn lại."""
        target_map = payload.get('map_name') or payload.get('mapName') or payload.get('map') or payload.get('active_map') or active_map
        force = bool(payload.get('force', payload.get('override_safety', False)))

        v_sub = payload.get('vertex') or payload.get('point') or payload.get('waypoint') or payload.get('data')
        data_src = payload
        if isinstance(v_sub, dict):
            data_src = {**payload, **v_sub}

        target_idx_val = None
        for key in ['index', 'idx', 'vertex_index', 'vertexIndex', 'point_index', 'pointIndex', 'node_index', 'nodeIndex']:
            if key in data_src and data_src[key] is not None:
                v = data_src[key]
                if isinstance(v, int) and not isinstance(v, bool):
                    target_idx_val = v
                    break
                v_str = str(v).strip()
                if v_str.isdigit():
                    target_idx_val = int(v_str)
                    break

        target_name = None
        full_data_tmp, _, _, vertices_tmp, _ = self.load_graph(target_map, active_map)
        for key in [
            'name', 'vertex_name', 'vertexName', 'vertex_id', 'vertexId', 'goal_id', 'goalId',
            'waypoint_name', 'waypointName', 'point_name', 'pointName', 'node_name', 'nodeName',
            'target_name', 'targetName', 'station_name', 'stationName', 'waypoint_id', 'waypointId',
            'node_id', 'nodeId', 'station_id', 'stationId', 'label', 'node', 'waypoint', 'vertex',
            'target', 'point', 'id', 'data'
        ]:
            if key in data_src and data_src[key] is not None:
                v = data_src[key]
                if isinstance(v, dict):
                    res_idx, res_n = resolve_vertex_index(v, vertices_tmp)
                    if res_n:
                        target_name = res_n
                        break
                    if res_idx is not None:
                        target_idx_val = res_idx
                        break
                elif v is not None:
                    v_str = str(v).strip()
                    target_name = v_str
                    break

        full_data, map_name, level_name, vertices, lanes = self.load_graph(target_map, active_map)

        del_idx = None
        x_req = data_src.get('x')
        y_req = data_src.get('y')
        if x_req is not None and y_req is not None:
            try:
                fx, fy = float(x_req), float(y_req)
                min_d = float('inf')
                best_i = None
                for i, v in enumerate(vertices):
                    d = math.hypot(float(v[0]) - fx, float(v[1]) - fy)
                    if d < min_d:
                        min_d = d
                        best_i = i
                if best_i is not None and min_d < 0.5:
                    del_idx = best_i
            except (ValueError, TypeError):
                pass

        if del_idx is None and target_idx_val is not None:
            del_idx, _ = resolve_vertex_index(target_idx_val, vertices)
        elif del_idx is None and target_name is not None:
            del_idx, _ = resolve_vertex_index(target_name, vertices)

        if del_idx is None and (target_name is not None or target_idx_val is not None):
            query = target_name if target_name is not None else target_idx_val
            for yaml_f in self.maps_dir.glob('*_topology.yaml'):
                cand_map = yaml_f.stem.replace('_topology', '')
                if cand_map != target_map:
                    cand_data, _, cand_lvl, cand_verts, cand_lanes = self.load_graph(cand_map, active_map)
                    cand_idx, _ = resolve_vertex_index(query, cand_verts)
                    if cand_idx is not None:
                        target_map = cand_map
                        full_data, level_name, vertices, lanes = cand_data, cand_lvl, cand_verts, cand_lanes
                        del_idx = cand_idx
                        break

        if del_idx is None:
            target_query = target_name if target_name is not None else (target_idx_val if target_idx_val is not None else payload)
            raise KeyError(f"Vertex '{target_query}' not found in topology for map '{target_map}'")

        safe, safe_msg = self.check_safety(vertices, lanes, robot_x, robot_y, robot_status, target_vertex_idx=del_idx, force=force)
        if not safe:
            raise RuntimeError(safe_msg)

        v_info = vertices[del_idx]
        v_name = v_info[2].get('name', f'idx_{del_idx}') if len(v_info) > 2 and isinstance(v_info[2], dict) else f'idx_{del_idx}'

        filtered_lanes = [l for l in lanes if int(l[0]) != del_idx and int(l[1]) != del_idx]
        removed_lanes_count = len(lanes) - len(filtered_lanes)

        del vertices[del_idx]

        reindexed_lanes = []
        for l in filtered_lanes:
            u, v = int(l[0]), int(l[1])
            new_u = u - 1 if u > del_idx else u
            new_v = v - 1 if v > del_idx else v
            opts = l[2] if len(l) > 2 and isinstance(l[2], dict) else {}
            reindexed_lanes.append([new_u, new_v, opts])

        valid, err = self.validate(vertices, reindexed_lanes)
        if not valid:
            raise ValueError(f'Validation failed after delete_vertex: {err}')

        full_data['levels'][level_name]['vertices'] = vertices
        full_data['levels'][level_name]['lanes'] = reindexed_lanes

        success, save_msg = self.save_atomic(full_data, map_name=target_map, active_map=active_map)
        if not success:
            raise IOError(f'Atomic save failed: {save_msg}')

        try:
            from mybot_goal.goal_storage import GoalStorage
            g_storage = GoalStorage()
            g_storage.delete_goal(target_map, v_name)
        except Exception:
            pass

        return {
            'status': 'ok',
            'action': 'delete_vertex',
            'map_name': target_map,
            'deleted_vertex': v_name,
            'message': f"Successfully deleted vertex '{v_name}' from map '{target_map}' (removed {removed_lanes_count} connected lanes)",
            'vertices_count': len(vertices),
            'lanes_count': len(reindexed_lanes)
        }

    def add_lane(self, payload: dict, active_map: str = 'map_lab20') -> dict:
        """Thêm hoặc cập nhật một làn đường (lane/edge) nối 2 điểm trong bản đồ Topology."""
        target_map = payload.get('map_name') or active_map

        from_v = None
        for k in ['from', 'start', 'start_vertex', 'from_vertex', 'from_node', 'start_node', 'from_point', 'start_point', 'node1', 'node_1', 'point1', 'point_1', 'u', 'source', 'start_idx', 'from_idx', 'start_name', 'from_name', 'first', 'v1']:
            if k in payload and payload[k] is not None:
                from_v = payload[k]
                break

        to_v = None
        for k in ['to', 'end', 'end_vertex', 'to_vertex', 'to_node', 'end_node', 'to_point', 'end_point', 'node2', 'node_2', 'point2', 'point_2', 'v', 'target', 'end_idx', 'to_idx', 'end_name', 'to_name', 'second', 'v2']:
            if k in payload and payload[k] is not None:
                to_v = payload[k]
                break

        speed_limit = None
        if 'speed_limit' in payload and payload['speed_limit'] is not None:
            try:
                speed_limit = float(payload['speed_limit'])
            except (ValueError, TypeError):
                pass

        is_bidirectional = None
        for b_key in ['is_bidirectional', 'bidirectional', 'bidi']:
            if b_key in payload and payload[b_key] is not None:
                is_bidirectional = bool(payload[b_key])
                break

        for container_key in ['lane', 'edge', 'link', 'route', 'segment', 'data']:
            if container_key in payload and payload[container_key] is not None:
                cont = payload[container_key]
                if isinstance(cont, (list, tuple)) and len(cont) >= 2:
                    if from_v is None:
                        from_v = cont[0]
                    if to_v is None:
                        to_v = cont[1]
                    if len(cont) > 2 and isinstance(cont[2], dict):
                        if speed_limit is None and 'speed_limit' in cont[2]:
                            try:
                                speed_limit = float(cont[2]['speed_limit'])
                            except (ValueError, TypeError):
                                pass
                        if is_bidirectional is None:
                            if 'is_bidirectional' in cont[2]:
                                is_bidirectional = bool(cont[2]['is_bidirectional'])
                            elif 'bidirectional' in cont[2]:
                                is_bidirectional = bool(cont[2]['bidirectional'])
                elif isinstance(cont, dict):
                    if from_v is None:
                        for k in ['from', 'start', 'u', 'source', 'start_vertex', 'from_vertex', 'start_name', 'from_name']:
                            if k in cont and cont[k] is not None:
                                from_v = cont[k]
                                break
                    if to_v is None:
                        for k in ['to', 'end', 'v', 'target', 'end_vertex', 'to_vertex', 'end_name', 'to_name']:
                            if k in cont and cont[k] is not None:
                                to_v = cont[k]
                                break
                    if speed_limit is None and 'speed_limit' in cont:
                        try:
                            speed_limit = float(cont['speed_limit'])
                        except (ValueError, TypeError):
                            pass
                    if is_bidirectional is None:
                        if 'is_bidirectional' in cont:
                            is_bidirectional = bool(cont['is_bidirectional'])
                        elif 'bidirectional' in cont:
                            is_bidirectional = bool(cont['bidirectional'])
                elif isinstance(cont, str) and any(sep in cont for sep in ['->', '-', '_', ' ']):
                    sep = '->' if '->' in cont else ('-' if '-' in cont else ('_' if '_' in cont else ' '))
                    parts = cont.split(sep)
                    if len(parts) >= 2:
                        if from_v is None:
                            from_v = parts[0].strip()
                        if to_v is None:
                            to_v = parts[1].strip()

        if is_bidirectional is None:
            is_bidirectional = True
        if speed_limit is None:
            speed_limit = 0.0

        full_data, map_name, level_name, vertices, lanes = self.load_graph(target_map, active_map)

        u_idx, u_name = resolve_vertex_index(from_v, vertices)
        v_idx, v_name = resolve_vertex_index(to_v, vertices)

        if u_idx is None or v_idx is None:
            raise KeyError(f"Cannot find vertices for lane from '{from_v}' to '{to_v}' in map '{target_map}'")

        if u_idx == v_idx:
            raise ValueError(f"Cannot add self-loop lane from '{u_name}' ({u_idx}) to itself ({v_idx})")

        lane_opts = {'speed_limit': speed_limit}

        forward_found = False
        for l in lanes:
            if int(l[0]) == u_idx and int(l[1]) == v_idx:
                if len(l) > 2 and isinstance(l[2], dict):
                    l[2]['speed_limit'] = speed_limit
                else:
                    if len(l) > 2:
                        l[2] = lane_opts
                    else:
                        l.append(lane_opts)
                forward_found = True
                break
        if not forward_found:
            lanes.append([u_idx, v_idx, lane_opts.copy()])

        if is_bidirectional:
            reverse_found = False
            for l in lanes:
                if int(l[0]) == v_idx and int(l[1]) == u_idx:
                    if len(l) > 2 and isinstance(l[2], dict):
                        l[2]['speed_limit'] = speed_limit
                    else:
                        if len(l) > 2:
                            l[2] = lane_opts
                        else:
                            l.append(lane_opts)
                    reverse_found = True
                    break
            if not reverse_found:
                lanes.append([v_idx, u_idx, lane_opts.copy()])

        valid, err = self.validate(vertices, lanes)
        if not valid:
            raise ValueError(f'Validation failed after add_lane: {err}')

        full_data['levels'][level_name]['vertices'] = vertices
        full_data['levels'][level_name]['lanes'] = lanes

        success, save_msg = self.save_atomic(full_data, map_name=target_map, active_map=active_map)
        if not success:
            raise IOError(f'Atomic save failed: {save_msg}')

        bidi_str = ' (bidirectional)' if is_bidirectional else ' (one-way)'
        return {
            'status': 'ok',
            'action': 'add_lane',
            'map_name': target_map,
            'message': f"Added lane between '{u_name}' ({u_idx}) and '{v_name}' ({v_idx}){bidi_str} for map '{target_map}'",
            'lanes_count': len(lanes)
        }

    def delete_lane(
        self,
        payload: dict,
        active_map: str = 'map_lab20',
        robot_x: float = 0.0,
        robot_y: float = 0.0,
        robot_status: str = 'idle'
    ) -> dict:
        """Xóa làn đường giữa 2 điểm (hỗ trợ xóa 2 chiều hoặc xóa 1 chiều tùy chọn)."""
        target_map = payload.get('map_name') or payload.get('mapName') or payload.get('map') or payload.get('active_map') or active_map
        force = bool(payload.get('force', payload.get('override_safety', False)))

        from_v = None
        for k in [
            'from', 'start', 'source', 'start_vertex', 'from_vertex', 'fromVertex', 'startVertex',
            'from_node', 'start_node', 'fromNode', 'startNode', 'from_point', 'start_point',
            'fromPoint', 'startPoint', 'source_node', 'sourceNode', 'node1', 'node_1', 'point1',
            'point_1', 'u', 'start_idx', 'from_idx', 'start_name', 'from_name', 'first', 'v1'
        ]:
            if k in payload and payload[k] is not None:
                from_v = payload[k]
                break

        to_v = None
        for k in [
            'to', 'end', 'target', 'end_vertex', 'to_vertex', 'toVertex', 'endVertex',
            'to_node', 'end_node', 'toNode', 'endNode', 'to_point', 'end_point',
            'toPoint', 'endPoint', 'target_node', 'targetNode', 'node2', 'node_2', 'point2',
            'point_2', 'v', 'end_idx', 'to_idx', 'end_name', 'to_name', 'second', 'v2'
        ]:
            if k in payload and payload[k] is not None:
                to_v = payload[k]
                break

        is_bidirectional = None
        for b_key in ['is_bidirectional', 'isBidirectional', 'bidirectional', 'bidi']:
            if b_key in payload and payload[b_key] is not None:
                is_bidirectional = bool(payload[b_key])
                break
        if payload.get('unidirectional') is True or payload.get('is_unidirectional') is True or payload.get('isUnidirectional') is True:
            is_bidirectional = False

        for container_key in ['lane', 'edge', 'link', 'route', 'segment', 'data', 'id', 'lane_id', 'laneId', 'edge_id', 'edgeId', 'name']:
            if container_key in payload and payload[container_key] is not None:
                cont = payload[container_key]
                if isinstance(cont, (list, tuple)) and len(cont) >= 2:
                    if from_v is None:
                        from_v = cont[0]
                    if to_v is None:
                        to_v = cont[1]
                elif isinstance(cont, dict):
                    if from_v is None:
                        for k in [
                            'from', 'start', 'source', 'start_vertex', 'from_vertex', 'fromVertex',
                            'startVertex', 'from_node', 'start_node', 'fromNode', 'startNode',
                            'from_point', 'start_point', 'fromPoint', 'startPoint', 'source_node',
                            'sourceNode', 'start_name', 'from_name', 'u', 'first', 'v1'
                        ]:
                            if k in cont and cont[k] is not None:
                                from_v = cont[k]
                                break
                    if to_v is None:
                        for k in [
                            'to', 'end', 'target', 'end_vertex', 'to_vertex', 'toVertex',
                            'endVertex', 'to_node', 'end_node', 'toNode', 'endNode',
                            'to_point', 'end_point', 'toPoint', 'endPoint', 'target_node',
                            'targetNode', 'end_name', 'to_name', 'v', 'second', 'v2'
                        ]:
                            if k in cont and cont[k] is not None:
                                to_v = cont[k]
                                break
                    if is_bidirectional is None:
                        if 'is_bidirectional' in cont:
                            is_bidirectional = bool(cont['is_bidirectional'])
                        elif 'isBidirectional' in cont:
                            is_bidirectional = bool(cont['isBidirectional'])
                        elif 'bidirectional' in cont:
                            is_bidirectional = bool(cont['bidirectional'])
                        elif cont.get('unidirectional') is True or cont.get('is_unidirectional') is True or cont.get('isUnidirectional') is True:
                            is_bidirectional = False
                elif isinstance(cont, str) and any(sep in cont for sep in ['->', '-', '_', ' ']):
                    sep = '->' if '->' in cont else ('-' if '-' in cont else ('_' if '_' in cont else ' '))
                    parts = cont.split(sep)
                    if len(parts) >= 2:
                        if from_v is None:
                            from_v = parts[0].strip()
                        if to_v is None:
                            to_v = parts[1].strip()

        if is_bidirectional is None:
            is_bidirectional = True

        lane_idx = None
        for k in ['index', 'lane_index', 'laneIndex', 'idx', 'id']:
            if k in payload and payload[k] is not None:
                val = payload[k]
                if isinstance(val, int) and not isinstance(val, bool):
                    lane_idx = val
                    break
                val_str = str(val).strip()
                if val_str.isdigit():
                    lane_idx = int(val_str)
                    break

        full_data, map_name, level_name, vertices, lanes = self.load_graph(target_map, active_map)

        u_idx, v_idx = None, None
        u_name, v_name = '', ''

        if from_v is not None and to_v is not None:
            u_idx, u_name = resolve_vertex_index(from_v, vertices)
            v_idx, v_name = resolve_vertex_index(to_v, vertices)

            if u_idx is None or v_idx is None:
                for yaml_f in self.maps_dir.glob('*_topology.yaml'):
                    cand_map = yaml_f.stem.replace('_topology', '')
                    if cand_map != target_map:
                        cand_data, _, cand_lvl, cand_verts, cand_lanes = self.load_graph(cand_map, active_map)
                        cand_u_idx, cand_u_name = resolve_vertex_index(from_v, cand_verts)
                        cand_v_idx, cand_v_name = resolve_vertex_index(to_v, cand_verts)
                        if cand_u_idx is not None and cand_v_idx is not None:
                            target_map = cand_map
                            full_data, level_name, vertices, lanes = cand_data, cand_lvl, cand_verts, cand_lanes
                            u_idx, u_name = cand_u_idx, cand_u_name
                            v_idx, v_name = cand_v_idx, cand_v_name
                            break

            if u_idx is None or v_idx is None:
                raise KeyError(f"Cannot find lane endpoints '{from_v}' -> '{to_v}' in map '{target_map}'")
            target_lane = [u_idx, v_idx]

        elif lane_idx is not None and 0 <= lane_idx < len(lanes):
            target_lane = lanes[lane_idx]
            u_idx, v_idx = int(target_lane[0]), int(target_lane[1])
            u_name = vertices[u_idx][2].get('name', f'idx_{u_idx}') if u_idx < len(vertices) and len(vertices[u_idx]) > 2 and isinstance(vertices[u_idx][2], dict) else f'idx_{u_idx}'
            v_name = vertices[v_idx][2].get('name', f'idx_{v_idx}') if v_idx < len(vertices) and len(vertices[v_idx]) > 2 and isinstance(vertices[v_idx][2], dict) else f'idx_{v_idx}'
        else:
            raise KeyError(f"Missing endpoints or index to delete lane for map '{target_map}'")

        safe, safe_msg = self.check_safety(vertices, lanes, robot_x, robot_y, robot_status, target_lane=target_lane, force=force)
        if not safe:
            raise RuntimeError(safe_msg)

        new_lanes = []
        deleted_count = 0
        for l in lanes:
            l_u, l_v = int(l[0]), int(l[1])
            if l_u == u_idx and l_v == v_idx:
                deleted_count += 1
            elif is_bidirectional and l_u == v_idx and l_v == u_idx:
                deleted_count += 1
            else:
                new_lanes.append(l)

        if deleted_count == 0:
            raise KeyError(f"Lane ({u_name} - {v_name}) not found in map '{target_map}'")

        valid, err = self.validate(vertices, new_lanes)
        if not valid:
            raise ValueError(f'Validation failed after delete_lane: {err}')

        full_data['levels'][level_name]['vertices'] = vertices
        full_data['levels'][level_name]['lanes'] = new_lanes

        success, save_msg = self.save_atomic(full_data, map_name=target_map, active_map=active_map)
        if not success:
            raise IOError(f'Atomic save failed: {save_msg}')

        return {
            'status': 'ok',
            'action': 'delete_lane',
            'map_name': target_map,
            'deleted_lanes_count': deleted_count,
            'message': f"Successfully deleted lane between '{u_name}' ({u_idx}) and '{v_name}' ({v_idx}) for map '{target_map}'",
            'lanes_count': len(new_lanes)
        }

    def full_save(self, payload: dict, active_map: str = 'map_lab20') -> dict:
        """Lưu toàn bộ đồ thị Topology hoàn chỉnh."""
        target_map = payload.get('map_name') or active_map
        level_name = payload.get('level_name') or payload.get('level') or 'L1'

        top_obj = payload.get('topology') if isinstance(payload.get('topology'), dict) else payload
        levels_data = top_obj.get('levels') or payload.get('levels')
        lifts = top_obj.get('lifts') if 'lifts' in top_obj else payload.get('lifts', {})
        doors = top_obj.get('doors') if 'doors' in top_obj else payload.get('doors', {})
        building_name = top_obj.get('building_name') or payload.get('building_name')

        has_explicit_data = (levels_data is not None) or ('vertices' in top_obj) or ('vertices' in payload)

        if not has_explicit_data:
            full_data, map_name, level_name, v_list, l_list = self.load_graph(target_map, active_map)
            yaml_content = full_data
        elif levels_data and isinstance(levels_data, dict):
            yaml_content = {'levels': levels_data, 'lifts': lifts, 'doors': doors}
            if building_name:
                yaml_content['building_name'] = building_name
            first_level = next(iter(levels_data.values()), {})
            v_list = first_level.get('vertices', [])
            l_list = first_level.get('lanes', [])
        else:
            vertices = top_obj.get('vertices') if 'vertices' in top_obj else payload.get('vertices', [])
            lanes = top_obj.get('lanes') if 'lanes' in top_obj else payload.get('lanes', [])

            formatted_vertices = []
            for i, v in enumerate(vertices):
                if isinstance(v, (list, tuple)) and len(v) >= 2:
                    props = v[2].copy() if len(v) > 2 and isinstance(v[2], dict) else {}
                    norm_p = {
                        'name': str(props.get('name') or f'P{i+1}').strip(),
                        'is_charger': bool(props.get('is_charger', False)),
                        'is_parking_spot': bool(props.get('is_parking_spot', False)),
                        'is_pickup': bool(props.get('is_pickup', False)),
                        'is_destination': bool(props.get('is_destination', False)),
                        'is_boost': bool(props.get('is_boost', False)),
                        'yaw': float(props.get('yaw', 0.0))
                    }
                    props.update(norm_p)
                    formatted_vertices.append([float(v[0]), float(v[1]), props])
                elif isinstance(v, dict):
                    x = float(v.get('x', 0.0))
                    y = float(v.get('y', 0.0))
                    props = v.copy()
                    norm_p = {
                        'name': str(v.get('name') or v.get('goal_id') or f'P{i+1}').strip(),
                        'is_parking_spot': bool(v.get('is_parking_spot', False)),
                        'is_charger': bool(v.get('is_charger', False)),
                        'is_pickup': bool(v.get('is_pickup', False)),
                        'is_destination': bool(v.get('is_destination', False)),
                        'is_boost': bool(v.get('is_boost', False)),
                        'yaw': float(v.get('yaw', 0.0))
                    }
                    props.update(norm_p)
                    formatted_vertices.append([x, y, props])

            formatted_lanes = []
            for l in lanes:
                if isinstance(l, (list, tuple)) and len(l) >= 2:
                    opts = l[2] if len(l) > 2 and isinstance(l[2], dict) else {}
                    formatted_lanes.append([int(l[0]), int(l[1]), opts])
                elif isinstance(l, dict):
                    u_val = l.get('from', l.get('start', l.get('u', l.get('start_vertex'))))
                    v_val = l.get('to', l.get('end', l.get('v', l.get('end_vertex'))))
                    u_idx, _ = resolve_vertex_index(u_val, formatted_vertices)
                    v_idx, _ = resolve_vertex_index(v_val, formatted_vertices)
                    if u_idx is not None and v_idx is not None and u_idx != v_idx:
                        speed = float(l.get('speed_limit', 0.0))
                        props = {'speed_limit': speed}
                        formatted_lanes.append([u_idx, v_idx, props])
                        bidi = bool(l.get('is_bidirectional', l.get('bidirectional', False)))
                        if bidi and not any(int(ol[0]) == v_idx and int(ol[1]) == u_idx for ol in formatted_lanes):
                            formatted_lanes.append([v_idx, u_idx, props])

            yaml_content = {
                'levels': {
                    level_name: {
                        'vertices': formatted_vertices,
                        'lanes': formatted_lanes
                    }
                },
                'lifts': lifts,
                'doors': doors
            }
            if building_name:
                yaml_content['building_name'] = building_name
            v_list = formatted_vertices
            l_list = formatted_lanes

        valid, err = self.validate(v_list, l_list)
        if not valid:
            raise ValueError(f'Full save validation failed: {err}')

        success, msg = self.save_atomic(yaml_content, map_name=target_map, active_map=active_map)
        if not success:
            raise IOError(f'Atomic save failed: {msg}')

        return {
            'status': 'ok',
            'action': 'save_topology',
            'map_name': target_map,
            'level_name': level_name,
            'vertices_count': len(v_list),
            'lanes_count': len(l_list),
            'message': f"Successfully saved topology for map '{target_map}' ({len(v_list)} vertices, {len(l_list)} lanes)"
        }
