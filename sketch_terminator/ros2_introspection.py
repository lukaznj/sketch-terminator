"""Pre-fetch ROS2 environment state at startup.

Scans topics, services, actions, and their interface definitions so the
agent already knows everything available without needing tool calls.
"""

import subprocess
from dataclasses import dataclass, field
from rich.console import Console

console = Console()

@dataclass
class ROS2State:
    """Snapshot of the current ROS2 environment."""
    topics: dict[str, str] = field(default_factory=dict)       # name -> type
    services: dict[str, str] = field(default_factory=dict)     # name -> type
    actions: dict[str, str] = field(default_factory=dict)      # name -> type
    interfaces: dict[str, str] = field(default_factory=dict)   # type -> definition

def _run(cmd: list[str], timeout: int = 15) -> str:
    """Run a subprocess and return stdout, or empty string on failure."""
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        if result.returncode == 0:
            return result.stdout.strip()
    except Exception:
        pass
    return ""

def _parse_typed_list(output: str) -> dict[str, str]:
    """Parse output from `ros2 <thing> list -t` into {name: type}.

    Lines look like: ``/topic_name [msg/Type]``
    """
    items: dict[str, str] = {}
    for line in output.splitlines():
        line = line.strip()
        if not line:
            continue
        if "[" in line and "]" in line:
            name, rest = line.split("[", 1)
            msg_type = rest.rstrip("]").strip()
            items[name.strip()] = msg_type
        else:
            items[line] = ""
    return items

def scan_ros2_environment() -> ROS2State:
    """Scan the live ROS2 graph and return a full state snapshot.

    Collects topics, services, actions and resolves all unique interface
    definitions. Displays a rich progress spinner while working.
    """
    state = ROS2State()

    with console.status("[bold blue]Scanning ROS2 environment...") as status:
        # --- Topics ---
        status.update("[bold blue]Discovering topics...")
        raw = _run(["ros2", "topic", "list", "-t"])
        if raw:
            state.topics = _parse_typed_list(raw)

        # --- Services ---
        status.update("[bold blue]Discovering services...")
        raw = _run(["ros2", "service", "list", "-t"])
        if raw:
            state.services = _parse_typed_list(raw)

        # --- Actions ---
        status.update("[bold blue]Discovering actions...")
        raw = _run(["ros2", "action", "list", "-t"])
        if raw:
            state.actions = _parse_typed_list(raw)

        # --- Interface definitions for every unique type ---
        all_types: set[str] = set()
        for mapping in (state.topics, state.services, state.actions):
            all_types.update(t for t in mapping.values() if t)

        total = len(all_types)
        for idx, iface_type in enumerate(sorted(all_types), 1):
            status.update(
                f"[bold blue]Fetching interface {idx}/{total}: {iface_type}"
            )
            defn = _run(["ros2", "interface", "show", iface_type])
            if defn:
                state.interfaces[iface_type] = defn

    # Print summary
    console.print(
        f"  [green]Discovered:[/green] "
        f"{len(state.topics)} topics, "
        f"{len(state.services)} services, "
        f"{len(state.actions)} actions, "
        f"{len(state.interfaces)} interface definitions"
    )

    return state

def format_state_for_prompt(state: ROS2State) -> str:
    """Format the ROS2 state into a string suitable for the system prompt."""
    parts: list[str] = []

    parts.append("=== AVAILABLE ROS2 TOPICS ===")
    if state.topics:
        for name, msg_type in sorted(state.topics.items()):
            parts.append(f"  {name}  [{msg_type}]")
    else:
        parts.append("  (none discovered)")

    parts.append("\n=== AVAILABLE ROS2 SERVICES ===")
    if state.services:
        for name, srv_type in sorted(state.services.items()):
            parts.append(f"  {name}  [{srv_type}]")
    else:
        parts.append("  (none discovered)")

    parts.append("\n=== AVAILABLE ROS2 ACTIONS ===")
    if state.actions:
        for name, action_type in sorted(state.actions.items()):
            parts.append(f"  {name}  [{action_type}]")
    else:
        parts.append("  (none discovered)")

    parts.append("\n=== INTERFACE DEFINITIONS ===")
    if state.interfaces:
        for iface_type, defn in sorted(state.interfaces.items()):
            parts.append(f"\n--- {iface_type} ---")
            parts.append(defn)
    else:
        parts.append("  (none fetched)")

    return "\n".join(parts)
