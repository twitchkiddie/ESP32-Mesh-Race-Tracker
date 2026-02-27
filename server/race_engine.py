"""
race_engine.py — Race-aware layer for ESP32 Mesh Crew Tracker

Wraps the live GPS node state from SerialBridge and adds:
  • Course definition  (start line, ordered waypoints, finish line)
  • Line / mark crossing detection
  • Per-boat race state  (pre_race → racing → finished | dnf)
  • Rankings by distance along course / elapsed time
  • Time and distance gaps to leader
  • ETA to finish (uses live GPS speed)

Thread-safe: all state mutations use a single lock.
"""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple


# ===========================================================================
# Geometry helpers
# ===========================================================================

def _haversine(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """Great-circle distance in metres between (lat, lon) tuples."""
    R = 6_371_000.0
    φ1, λ1 = math.radians(a[0]), math.radians(a[1])
    φ2, λ2 = math.radians(b[0]), math.radians(b[1])
    dφ, dλ = φ2 - φ1, λ2 - λ1
    t = math.sin(dφ / 2) ** 2 + math.cos(φ1) * math.cos(φ2) * math.sin(dλ / 2) ** 2
    return R * 2 * math.asin(math.sqrt(max(0.0, t)))


def _side(lp1: Tuple, lp2: Tuple, pt: Tuple) -> float:
    """
    Z-component of (lp2-lp1) × (pt-lp1) in lon-lat space.
    Positive = left of line, negative = right, zero = on line.
    """
    ax, ay = lp2[1] - lp1[1], lp2[0] - lp1[0]
    bx, by = pt[1]  - lp1[1], pt[0]  - lp1[0]
    return ax * by - ay * bx


def _crosses(lp1: Tuple, lp2: Tuple, prev: Tuple, curr: Tuple) -> bool:
    """True when the path prev→curr crosses the segment lp1–lp2."""
    if _side(lp1, lp2, prev) * _side(lp1, lp2, curr) >= 0:
        return False
    # Bbox guard: only fire if the midpoint of the crossing is within the
    # segment's extent (with 15% padding to handle edge-cases near endpoints).
    def _between(v, a, b, pad=0.15):
        lo, hi = sorted([a, b])
        d = (hi - lo) * pad or 1e-6
        return lo - d <= v <= hi + d
    mid = ((prev[0] + curr[0]) / 2, (prev[1] + curr[1]) / 2)
    return _between(mid[0], lp1[0], lp2[0]) and _between(mid[1], lp1[1], lp2[1])


def _fmt_time(seconds: Optional[float]) -> str:
    """Format seconds as MM:SS or H:MM:SS."""
    if seconds is None:
        return "--:--"
    t = max(0, int(seconds))
    h, r = divmod(t, 3600)
    m, s = divmod(r, 60)
    return f"{h}:{m:02d}:{s:02d}" if h else f"{m:02d}:{s:02d}"


# ===========================================================================
# Course data model
# ===========================================================================

@dataclass
class GeoPoint:
    lat:  float
    lon:  float
    name: str = ""

    def t(self) -> Tuple[float, float]:
        return (self.lat, self.lon)

    def to_dict(self) -> dict:
        return {"lat": self.lat, "lon": self.lon, "name": self.name}

    @staticmethod
    def from_dict(d: dict) -> "GeoPoint":
        return GeoPoint(float(d["lat"]), float(d["lon"]), d.get("name", ""))


@dataclass
class CourseLine:
    """A start or finish line defined by two GPS marks (port & starboard)."""
    p1:   GeoPoint
    p2:   GeoPoint
    name: str = "Line"

    @property
    def mid(self) -> Tuple[float, float]:
        return ((self.p1.lat + self.p2.lat) / 2,
                (self.p1.lon + self.p2.lon) / 2)

    def to_dict(self) -> dict:
        return {"p1": self.p1.to_dict(), "p2": self.p2.to_dict(), "name": self.name}

    @staticmethod
    def from_dict(d: dict) -> "CourseLine":
        return CourseLine(
            GeoPoint.from_dict(d["p1"]),
            GeoPoint.from_dict(d["p2"]),
            d.get("name", "Line"),
        )


@dataclass
class Course:
    start_line:  CourseLine
    finish_line: CourseLine
    waypoints:   List[GeoPoint] = field(default_factory=list)
    name:        str = "Race Course"

    @property
    def marks(self) -> List[Tuple[float, float]]:
        """Ordered (lat, lon) sequence: start mid → waypoints → finish mid."""
        return ([self.start_line.mid]
                + [w.t() for w in self.waypoints]
                + [self.finish_line.mid])

    @property
    def total_m(self) -> float:
        m = self.marks
        return sum(_haversine(m[i], m[i + 1]) for i in range(len(m) - 1))

    def to_dict(self) -> dict:
        m = self.marks
        return {
            "name":             self.name,
            "start_line":       self.start_line.to_dict(),
            "finish_line":      self.finish_line.to_dict(),
            "waypoints":        [w.to_dict() for w in self.waypoints],
            "marks":            [{"lat": p[0], "lon": p[1]} for p in m],
            "total_distance_m": round(self.total_m, 1),
        }

    @staticmethod
    def from_dict(d: dict) -> "Course":
        return Course(
            start_line  = CourseLine.from_dict(d["start_line"]),
            finish_line = CourseLine.from_dict(d["finish_line"]),
            waypoints   = [GeoPoint.from_dict(w) for w in d.get("waypoints", [])],
            name        = d.get("name", "Race Course"),
        )


# ===========================================================================
# Per-boat race state
# ===========================================================================

PRE_RACE = "pre_race"
RACING   = "racing"
FINISHED = "finished"
DNF      = "dnf"

WAYPOINT_RADIUS_M = 50.0   # metres — within this = mark rounded
MARK_COOLDOWN_S   = 30.0   # seconds — don't re-round same mark within this window


@dataclass
class BoatRace:
    mac:         str
    status:      str            = PRE_RACE
    rank:        int            = 0           # 1-based; 0 = unranked
    start_time:  Optional[float] = None
    finish_time: Optional[float] = None
    elapsed_s:   Optional[float] = None       # only set on finish
    next_mark:   int            = 1           # index into course.marks
    distance_m:  float          = 0.0
    gap_m:       float          = 0.0
    gap_s:       Optional[float] = None
    eta_s:       Optional[float] = None

    def __post_init__(self):
        # Internal tracking — not dataclass fields so they don't appear in comparisons
        self._prev: Optional[Tuple[float, float]] = None
        self._mark_t: float = 0.0             # timestamp of last mark rounding

    def running_elapsed(self) -> Optional[float]:
        if self.start_time is None:
            return None
        return self.elapsed_s if self.finish_time else (time.time() - self.start_time)

    def to_dict(self) -> dict:
        e = self.running_elapsed()
        return {
            "mac":         self.mac,
            "status":      self.status,
            "rank":        self.rank,
            "elapsed_s":   round(e, 1) if e is not None else None,
            "elapsed_fmt": _fmt_time(e),
            "next_mark":   self.next_mark,
            "distance_m":  round(self.distance_m, 1),
            "gap_m":       round(self.gap_m, 1),
            "gap_s":       round(self.gap_s, 1) if self.gap_s is not None else None,
            "gap_fmt":     (f"+{_fmt_time(self.gap_s)}" if self.gap_s else "Leader"),
            "eta_s":       round(self.eta_s, 0) if self.eta_s is not None else None,
            "eta_fmt":     _fmt_time(self.eta_s),
        }


# ===========================================================================
# Race Engine
# ===========================================================================

class RaceEngine:
    """
    Feed it the live nodes dict from SerialBridge every time it updates::

        engine = RaceEngine()
        engine.load_config('race_config.json')
        bridge.on_update(lambda nodes, gw: engine.update(nodes))

    Then query::

        state = engine.get_race_state(bridge.get_state()['nodes'])
    """

    def __init__(self):
        self._lock         = threading.Lock()
        self._course:       Optional[Course]       = None
        self._active_name:  str                    = ''
        self._courses:      Dict[str, Course]      = {}   # all saved courses
        self._boats:        Dict[str, BoatRace]    = {}

    # ------------------------------------------------------------------
    # Course management
    # ------------------------------------------------------------------

    def set_course(self, course: Course):
        """Save + activate a course.  Adds/replaces by name."""
        with self._lock:
            self._courses[course.name] = course
            self._course      = course
            self._active_name = course.name

    def get_course(self) -> Optional[Course]:
        with self._lock:
            return self._course

    def list_courses(self) -> list:
        """Return summary list of all saved courses."""
        with self._lock:
            return [
                {'name': n, 'active': n == self._active_name,
                 'total_m': round(c.total_m, 1)}
                for n, c in self._courses.items()
            ]

    def activate_course(self, name: str) -> bool:
        """Switch the active course by name; resets boat states."""
        with self._lock:
            if name not in self._courses:
                return False
            self._course      = self._courses[name]
            self._active_name = name
            self._boats.clear()
            return True

    def delete_course(self, name: str):
        """Remove a saved course.  If it was active, deactivate."""
        with self._lock:
            self._courses.pop(name, None)
            if self._active_name == name:
                self._course      = None
                self._active_name = ''
                self._boats.clear()

    def save_config(self, path: str):
        with self._lock:
            data = {
                'active':  self._active_name,
                'courses': {n: c.to_dict() for n, c in self._courses.items()},
            }
            Path(path).write_text(json.dumps(data, indent=2))

    def load_config(self, path: str) -> bool:
        try:
            d = json.loads(Path(path).read_text())
            with self._lock:
                if 'courses' in d:
                    # New multi-course format
                    self._courses = {n: Course.from_dict(c)
                                     for n, c in d['courses'].items()}
                    active = d.get('active', '')
                    if active and active in self._courses:
                        self._course      = self._courses[active]
                        self._active_name = active
                    elif self._courses:
                        # Fall back to first saved course
                        first = next(iter(self._courses))
                        self._course      = self._courses[first]
                        self._active_name = first
                else:
                    # Old single-course format — migrate automatically
                    course = Course.from_dict(d)
                    self._courses[course.name] = course
                    self._course      = course
                    self._active_name = course.name
            return True
        except Exception:
            return False

    # ------------------------------------------------------------------
    # Race control
    # ------------------------------------------------------------------

    def reset(self):
        """Clear all boat race states (course is kept)."""
        with self._lock:
            self._boats.clear()

    def flag_dnf(self, mac: str):
        with self._lock:
            if mac in self._boats:
                self._boats[mac].status = DNF
                self._boats[mac].rank = 0

    # ------------------------------------------------------------------
    # State update  (call every time SerialBridge fires a callback)
    # ------------------------------------------------------------------

    def update(self, nodes: dict):
        """
        nodes: the serialised node dict from SerialBridge.get_state()['nodes']
               i.e. {mac: node.to_dict(), ...}
        """
        with self._lock:
            if not self._course:
                return

            for mac, node in nodes.items():
                if node.get("role", -1) != 0:          # trackers only
                    continue
                if not node.get("has_gps", False):
                    continue
                lat, lon = node.get("lat", 0.0), node.get("lon", 0.0)
                if lat == 0.0 and lon == 0.0:
                    continue

                curr = (lat, lon)
                if mac not in self._boats:
                    self._boats[mac] = BoatRace(mac=mac)

                boat = self._boats[mac]
                if boat.status in (FINISHED, DNF):
                    boat._prev = curr
                    continue

                if boat._prev is not None:
                    self._process(boat, boat._prev, curr, self._course)

                boat._prev = curr

            self._rerank()

    # ------------------------------------------------------------------
    # Internal: position logic
    # ------------------------------------------------------------------

    def _process(self, boat: BoatRace, prev: Tuple, curr: Tuple, course: Course):
        marks = course.marks
        now   = time.time()

        # ── Pre-race: watch for start line crossing ──────────────────
        if boat.status == PRE_RACE:
            sl = course.start_line
            if _crosses(sl.p1.t(), sl.p2.t(), prev, curr):
                boat.status     = RACING
                boat.start_time = now
                boat.next_mark  = 1

        # ── Racing: advance through marks ────────────────────────────
        if boat.status == RACING:
            idx = boat.next_mark

            if idx < len(marks) - 1:
                # Heading to a waypoint — round by proximity
                if (_haversine(curr, marks[idx]) < WAYPOINT_RADIUS_M
                        and now - boat._mark_t > MARK_COOLDOWN_S):
                    boat.next_mark += 1
                    boat._mark_t = now

            elif idx == len(marks) - 1:
                # Heading to finish — watch for line crossing
                fl = course.finish_line
                if _crosses(fl.p1.t(), fl.p2.t(), prev, curr):
                    boat.status     = FINISHED
                    boat.finish_time = now
                    boat.elapsed_s  = now - boat.start_time
                    boat.next_mark  = len(marks)

            # Distance — never allow it to go backwards
            boat.distance_m = max(boat.distance_m,
                                  self._calc_distance(boat, curr, course))

    def _calc_distance(self, boat: BoatRace, pos: Tuple, course: Course) -> float:
        """Metres completed along the course from start line mid."""
        if boat.status == FINISHED:
            return course.total_m

        marks = course.marks
        done  = boat.next_mark - 1          # number of completed inter-mark legs

        # Sum completed legs
        d = sum(_haversine(marks[i], marks[i + 1]) for i in range(done))

        # Add partial distance on the current leg
        if done < len(marks) - 1:
            d += _haversine(marks[done], pos)

        return d

    # ------------------------------------------------------------------
    # Internal: rankings and gaps
    # ------------------------------------------------------------------

    def _rerank(self):
        course = self._course
        if not course:
            return

        boats    = list(self._boats.values())
        finished = sorted([b for b in boats if b.status == FINISHED],
                          key=lambda b: b.elapsed_s or float("inf"))
        racing   = sorted([b for b in boats if b.status == RACING],
                          key=lambda b: b.distance_m, reverse=True)

        # Assign ranks
        rank = 1
        for b in finished + racing:
            b.rank = rank
            rank  += 1
        for b in boats:
            if b.status in (PRE_RACE, DNF):
                b.rank = 0

        # Compute gaps
        leader = finished[0] if finished else (racing[0] if racing else None)
        if leader is None:
            return

        leader_dist = leader.distance_m
        leader_time = leader.running_elapsed() or 0.0
        avg_speed   = (leader_dist / leader_time) if leader_time > 0 else 0.0

        for b in boats:
            if b.status not in (RACING, FINISHED):
                continue
            if b is leader:
                b.gap_m = 0.0
                b.gap_s = None
            else:
                b.gap_m = max(0.0, leader_dist - b.distance_m)
                b.gap_s = (b.gap_m / avg_speed) if avg_speed > 0.1 else None

    # ------------------------------------------------------------------
    # Public: full state snapshot for the API
    # ------------------------------------------------------------------

    def get_race_state(self, nodes: dict) -> dict:
        """
        Returns the complete race state dict, augmented with live node data
        (color, speed, battery) for the frontend.
        nodes: serialised node dict from SerialBridge.get_state()['nodes']
        """
        with self._lock:
            course     = self._course
            boats_out  = []

            for mac, boat in self._boats.items():
                bd   = boat.to_dict()
                node = nodes.get(mac, {})

                # Augment with live fields from bridge
                bd.update({
                    "label":      node.get("label",      mac[-5:]),
                    "color":      node.get("color",      "#888888"),
                    "color_name": node.get("color_name", ""),
                    "lat":        node.get("lat"),
                    "lon":        node.get("lon"),
                    "speed_mps":  node.get("speed_mps",  0.0),
                    "speed_kmph": node.get("speed_kmph", 0.0),
                    "batt_pct":   node.get("batt_pct",   0),
                    "has_gps":    node.get("has_gps",    False),
                    "age_s":      node.get("age_s",      0),
                })

                # ETA uses live speed (more accurate than stored average)
                if boat.status == RACING and course:
                    spd       = node.get("speed_mps", 0.0)
                    remaining = course.total_m - boat.distance_m
                    if spd > 0.3 and remaining > 0:
                        bd["eta_s"]   = round(remaining / spd)
                        bd["eta_fmt"] = _fmt_time(bd["eta_s"])

                boats_out.append(bd)

            # Also show any trackers visible in the mesh but not yet in race
            # (e.g. no GPS lock yet) so the user can see they're connected
            seen = {b["mac"] for b in boats_out}
            for mac, node in nodes.items():
                if mac in seen:
                    continue
                if node.get("role", -1) != 0:   # trackers only
                    continue
                boats_out.append({
                    "mac":        mac,
                    "status":     PRE_RACE,
                    "rank":       0,
                    "elapsed_s":  0,
                    "elapsed_fmt":"--:--",
                    "distance_m": 0.0,
                    "gap_m":      0.0,
                    "gap_s":      None,
                    "gap_fmt":    "—",
                    "eta_s":      None,
                    "eta_fmt":    "—",
                    "label":      node.get("label",      mac[-5:]),
                    "color":      node.get("color",      "#888888"),
                    "color_name": node.get("color_name", ""),
                    "lat":        node.get("lat"),
                    "lon":        node.get("lon"),
                    "speed_mps":  node.get("speed_mps",  0.0),
                    "speed_kmph": node.get("speed_kmph", 0.0),
                    "batt_pct":   node.get("batt_pct",   0),
                    "has_gps":    node.get("has_gps",    False),
                    "age_s":      node.get("age_s",      0),
                })

            # Sort: ranked boats first (by rank), then unranked
            boats_out.sort(key=lambda b: b["rank"] if b["rank"] > 0 else 999)

            active   = any(b["status"] == RACING   for b in boats_out)
            finished = (len(boats_out) > 0
                        and all(b["status"] in (FINISHED, DNF) for b in boats_out))

            return {
                "course":        course.to_dict() if course else None,
                "boats":         boats_out,
                "race_active":   active,
                "race_finished": finished,
                "timestamp":     time.time(),
            }
