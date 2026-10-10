#!/usr/bin/env python3
"""GoalStorage module to save, update, delete and query goals saved in JSON files per map."""

import json
import os
from pathlib import Path
from mybot_common.constants import MAPS_DIR
from mybot_common.command_utils import sanitize_map_name


class GoalStorage:
    """Manages goal persistence on filesystem."""

    def __init__(self, storage_dir: Path = MAPS_DIR):
        self.storage_dir = storage_dir
        os.makedirs(self.storage_dir, exist_ok=True)

    def get_goal_file(self, map_name: str) -> Path:
        safe_name = sanitize_map_name(map_name) or "default"
        return self.storage_dir / f"{safe_name}_goals.json"

    def load_goals(self, map_name: str) -> list:
        """Load all goals for a map."""
        file_path = self.get_goal_file(map_name)
        if not file_path.is_file():
            return []
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                if isinstance(data, list):
                    return data
                if isinstance(data, dict) and "goals" in data:
                    return data["goals"]
        except Exception:
            pass
        return []

    def save_goals(self, map_name: str, goals: list) -> bool:
        """Save goal list to file."""
        file_path = self.get_goal_file(map_name)
        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump({"map_name": sanitize_map_name(map_name), "goals": goals}, f, indent=2)
            return True
        except Exception:
            return False

    def create_goal(self, map_name: str, goal_data: dict) -> dict:
        """Create a new goal.
        
        goal_data format:
        {
            "goal_id": "P1", (optional, auto-generated if missing)
            "name": "Point 1", (optional)
            "pose": {"x": 1.0, "y": 2.0, "theta": 0.0}
        }
        """
        goals = self.load_goals(map_name)
        goal_id = str(goal_data.get("goal_id") or goal_data.get("name") or f"G{len(goals) + 1}").strip()
        
        # Check if already exists
        for idx, existing in enumerate(goals):
            if str(existing.get("goal_id", "")).strip() == goal_id or str(existing.get("name", "")).strip() == goal_id:
                raise ValueError(f"Goal with ID or name '{goal_id}' already exists")

        new_goal = {
            "goal_id": goal_id,
            "name": goal_data.get("name") or goal_id,
            "pose": goal_data.get("pose", {"x": 0.0, "y": 0.0, "theta": 0.0}),
            "description": goal_data.get("description", "")
        }
        goals.append(new_goal)
        if not self.save_goals(map_name, goals):
            raise IOError("Failed to write goal to file")
        return new_goal

    def update_goal(self, map_name: str, goal_data: dict) -> dict:
        """Update an existing goal matching goal_id or name."""
        goals = self.load_goals(map_name)
        goal_id = str(goal_data.get("goal_id") or goal_data.get("name", "")).strip()
        if not goal_id:
            raise ValueError("Missing goal_id or name to update")

        found_index = -1
        for idx, existing in enumerate(goals):
            if str(existing.get("goal_id", "")).strip() == goal_id or str(existing.get("name", "")).strip() == goal_id:
                found_index = idx
                break

        if found_index == -1:
            raise KeyError(f"Goal '{goal_id}' not found for map '{map_name}'")

        current = goals[found_index]
        if "name" in goal_data:
            current["name"] = goal_data["name"]
        if "pose" in goal_data:
            current["pose"] = goal_data["pose"]
        if "description" in goal_data:
            current["description"] = goal_data["description"]

        goals[found_index] = current
        if not self.save_goals(map_name, goals):
            raise IOError("Failed to write updated goal to file")
        return current

    def delete_goal(self, map_name: str, goal_id_or_name: str) -> dict:
        """Delete goal matching goal_id or name."""
        goals = self.load_goals(map_name)
        target = str(goal_id_or_name).strip()
        if not target:
            raise ValueError("Missing goal_id or name to delete")

        new_goals = []
        deleted_item = None
        for existing in goals:
            if str(existing.get("goal_id", "")).strip() == target or str(existing.get("name", "")).strip() == target:
                deleted_item = existing
            else:
                new_goals.append(existing)

        if deleted_item is None:
            raise KeyError(f"Goal '{target}' not found for map '{map_name}'")

        if not self.save_goals(map_name, new_goals):
            raise IOError("Failed to write goals after deletion")
        return deleted_item
