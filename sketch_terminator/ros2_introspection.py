"""Snapshots the live ROS 2 graph so the agent starts already knowing it.

``agent.py`` can fold this into the system prompt instead of making the model
discover everything through tool calls. It is off by default: resolving every
interface definition means one ``ros2 interface show`` subprocess per type,
which costs roughly 15 seconds at startup.

Pass ``scan_environment=True`` to ``create_agent()`` to turn it on.
"""

import subprocess
from dataclasses import dataclass, field

from rich.console import Console

console = Console()

# Generous, because `ros2 topic list` blocks on discovery on a busy graph.
COMMAND_TIMEOUT_SEC = 15


@dataclass
class ROS2State:
    """What is currently on the ROS 2 graph."""
    topics: dict[str, str] = field(default_factory=dict)      # name -> message type
    services: dict[str, str] = field(default_factory=dict)    # name -> service type
    actions: dict[str, str] = field(default_factory=dict)     # name -> action type
    interfaces: dict[str, str] = field(default_factory=dict)  # type -> definition


def _run(cmd, timeout=COMMAND_TIMEOUT_SEC):
    """Run a command and return its stdout, or '' if anything goes wrong.

    Failures are swallowed on purpose: a missing interface definition should
    degrade the prompt, not stop the agent from starting.
    """
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return result.stdout.strip() if result.returncode == 0 else ''
    except Exception:
        return ''


def _parse_typed_list(output):
    """Parse ``ros2 <thing> list -t`` output into {name: type}.

    Lines look like ``/topic_name [msg/Type]``.
    """
    items = {}
    for line in output.splitlines():
        line = line.strip()
        if not line:
            continue
        if '[' in line and ']' in line:
            name, _, rest = line.partition('[')
            items[name.strip()] = rest.rstrip(']').strip()
        else:
            items[line] = ''
    return items


def scan_ros2_environment():
    """Walk the live graph and resolve every interface definition it uses."""
    state = ROS2State()

    with console.status('[bold blue]Scanning ROS 2 environment...') as status:
        for label, attr, command in (
            ('topics', 'topics', ['ros2', 'topic', 'list', '-t']),
            ('services', 'services', ['ros2', 'service', 'list', '-t']),
            ('actions', 'actions', ['ros2', 'action', 'list', '-t']),
        ):
            status.update(f'[bold blue]Discovering {label}...')
            raw = _run(command)
            if raw:
                setattr(state, attr, _parse_typed_list(raw))

        # One `ros2 interface show` per unique type -- this is the slow part.
        all_types = {
            type_name
            for mapping in (state.topics, state.services, state.actions)
            for type_name in mapping.values()
            if type_name
        }
        for index, interface_type in enumerate(sorted(all_types), 1):
            status.update(
                f'[bold blue]Fetching interface {index}/{len(all_types)}: {interface_type}'
            )
            definition = _run(['ros2', 'interface', 'show', interface_type])
            if definition:
                state.interfaces[interface_type] = definition

    console.print(
        f'  [green]Discovered:[/green] {len(state.topics)} topics, '
        f'{len(state.services)} services, {len(state.actions)} actions, '
        f'{len(state.interfaces)} interface definitions'
    )
    return state


def format_state_for_prompt(state):
    """Render a :class:`ROS2State` as plain text for the system prompt."""
    parts = []

    for heading, mapping in (
        ('AVAILABLE ROS2 TOPICS', state.topics),
        ('AVAILABLE ROS2 SERVICES', state.services),
        ('AVAILABLE ROS2 ACTIONS', state.actions),
    ):
        parts.append(f'=== {heading} ===')
        if mapping:
            parts += [f'  {name}  [{type_name}]' for name, type_name in sorted(mapping.items())]
        else:
            parts.append('  (none discovered)')
        parts.append('')

    parts.append('=== INTERFACE DEFINITIONS ===')
    if state.interfaces:
        for interface_type, definition in sorted(state.interfaces.items()):
            parts.append(f'\n--- {interface_type} ---')
            parts.append(definition)
    else:
        parts.append('  (none fetched)')

    return '\n'.join(parts)
