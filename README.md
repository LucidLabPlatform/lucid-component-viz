# lucid-component-viz

LUCID visualization component: supervises arena.py (pygame projection overlay) and TouchDesigner as subprocesses.

## Commands

| Command | Description |
|---------|-------------|
| `cmd/start_arena` | Start arena.py subprocess |
| `cmd/stop_arena` | Stop arena.py subprocess |
| `cmd/start_touchdesigner` | Start TouchDesigner |
| `cmd/stop_touchdesigner` | Stop TouchDesigner |
| `cmd/restart` | Stop and restart both processes |
| `cmd/reset` | Alias for restart |
| `cmd/ping` | Health check |

## State

```json
{
  "arena": {"running": true, "pid": 1234},
  "touchdesigner": {"running": true, "pid": 5678}
}
```

## Configuration

Registry config key: `touchdesigner_app` (default: `/Applications/TouchDesigner.app`)
