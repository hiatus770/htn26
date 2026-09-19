# Chess Robot Digital Twin

The service is the authoritative digital twin for a single physical robot operating a fixed set of real chess boards. It validates observed FEN positions, tracks each board independently, selects legal robot moves, records discrepancies, and serves a live monochrome dashboard.

## Run

```bash
uv venv engine/.venv
uv pip install --python engine/.venv/bin/python -r engine/requirements.txt
CHESS_GAME_COUNT=5 STOCKFISH_PATH=/usr/games/stockfish engine/.venv/bin/uvicorn engine.app:app --reload
```

Open `http://127.0.0.1:8000/` for the dashboard. `STOCKFISH_PATH` is optional for development only; without it the service selects a legal non-castling fallback move and leaves numerical evaluation unavailable.

`CHESS_DATABASE` overrides the default `engine/chess.db`. The configured game count is fixed for the lifetime of that database/session.

## Session flow

1. `POST /v1/session` once, with exactly the configured number of `{game_id, profile_id}` items.
2. Set up each physical board in the standard initial position and call `POST /v1/games/{id}/initial-board-confirmation` with the exact starting FEN. The response contains the robot opening move.
3. Execute it physically, then call `POST /v1/games/{id}/robot-move-completion`. Supply `observed_fen` when vision/sensing is available, or only `{ "completed": true }` for an acknowledgement-only workflow.
4. After the human moves, call `POST /v1/games/{id}/player-move` with UCI and the observed post-move FEN. A legal matching state yields the next robot move; a mismatch returns HTTP 409 and leaves the digital twin unchanged.

The robot is always White. Its castling candidates are excluded. The opponent may castle normally.

## Lifecycle controls

- `POST /v1/games/{id}/forfeit` with `{ "forfeiting_side": "robot" | "player", "reason": "optional" }` ends the active round.
- `POST /v1/games/{id}/restart` with optional `{ "profile_id": "...", "reason": "..." }` starts the next round in the same board slot. It always requires a new initial-board confirmation before play starts.

All actions are intentionally public: this API is the robot's digital-twin boundary. Production deployments should place it on the appropriate trusted network.

## Profiles and events

Edit `config/profiles.json` to add characters. Each profile has a stable ID, name, attitude, optional future `voice_id`, thinking time, candidate window, and chess style. Profiles are loaded at service start.

`GET /v1/session` is the dashboard snapshot API; `GET /v1/games/{id}` returns a full game. WebSocket clients connect to `/v1/events` and receive updates after every state transition. The dashboard reloads its REST snapshot after every event and reconnect.

## Results and validation

The service detects checkmate, stalemate, insufficient material, fivefold repetition, and the 75-move rule. It exposes claimable draws without automatically ending a game. Evaluation is from the robot's perspective: `>= +100cp` is winning, `<= -100cp` is losing, otherwise neutral; forced mates take precedence when supplied by the engine.

Every accepted move and lifecycle action is retained in SQLite. A mismatched FEN, illegal move, or out-of-order update becomes a discrepancy event rather than changing the authoritative position.
