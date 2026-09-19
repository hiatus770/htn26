"""FastAPI service for a physical chess robot's digital twin."""
from __future__ import annotations

import json
import os
import sqlite3
import uuid
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Annotated, Any, Literal

import chess
import chess.engine
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field, field_validator

ROOT = Path(__file__).resolve().parent
STARTING_FEN = chess.STARTING_FEN


def now() -> str:
    return datetime.now(timezone.utc).isoformat()


class GameSpec(BaseModel):
    game_id: str = Field(pattern=r"^[A-Za-z0-9_-]{1,64}$")
    profile_id: str


class SessionRequest(BaseModel):
    games: list[GameSpec]


class PlayerMoveRequest(BaseModel):
    move: str = Field(pattern=r"^[a-h][1-8][a-h][1-8][qrbn]?$")
    observed_fen: str


class RobotCompletionRequest(BaseModel):
    completed: bool = True
    observed_fen: str | None = None


class ForfeitRequest(BaseModel):
    forfeiting_side: Literal["robot", "player"]
    reason: str | None = Field(default=None, max_length=500)


class RestartRequest(BaseModel):
    profile_id: str | None = None
    reason: str | None = Field(default=None, max_length=500)


class InitialBoardRequest(BaseModel):
    observed_fen: str


class Profile(BaseModel):
    id: str
    display_name: str
    attitude: str
    voice_id: str | None = None
    think_time_ms: int = Field(ge=10, le=60000)
    candidate_window_cp: int = Field(ge=0, le=500)
    style: Literal["balanced", "tactical", "cautious", "positional"]


class Broker:
    def __init__(self) -> None:
        self.clients: set[WebSocket] = set()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self.clients.add(ws)

    def disconnect(self, ws: WebSocket) -> None:
        self.clients.discard(ws)

    async def publish(self, kind: str, payload: dict[str, Any]) -> None:
        message = {"type": kind, "at": now(), "data": payload}
        stale: list[WebSocket] = []
        for ws in list(self.clients):
            try:
                await ws.send_json(message)
            except Exception:
                stale.append(ws)
        for ws in stale:
            self.disconnect(ws)


class Store:
    def __init__(self, database_path: str) -> None:
        self.path = database_path

    def connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.path)
        conn.row_factory = sqlite3.Row
        return conn

    def init(self) -> None:
        with self.connect() as db:
            db.executescript("""
                PRAGMA journal_mode=WAL;
                CREATE TABLE IF NOT EXISTS session (
                  id INTEGER PRIMARY KEY CHECK (id = 1), game_count INTEGER NOT NULL,
                  initialized_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS games (
                  game_id TEXT PRIMARY KEY, profile_id TEXT NOT NULL, round_number INTEGER NOT NULL,
                  fen TEXT NOT NULL, status TEXT NOT NULL, result TEXT, result_type TEXT,
                  pending_move TEXT, pending_san TEXT, evaluation_cp INTEGER, evaluation_label TEXT,
                  last_error TEXT, created_at TEXT NOT NULL, updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS moves (
                  id INTEGER PRIMARY KEY AUTOINCREMENT, game_id TEXT NOT NULL, round_number INTEGER NOT NULL,
                  ply INTEGER NOT NULL, side TEXT NOT NULL, uci TEXT NOT NULL, san TEXT NOT NULL,
                  fen_after TEXT NOT NULL, is_capture INTEGER NOT NULL, captured_piece TEXT,
                  created_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS events (
                  id INTEGER PRIMARY KEY AUTOINCREMENT, game_id TEXT NOT NULL, round_number INTEGER NOT NULL,
                  event_type TEXT NOT NULL, detail TEXT NOT NULL, created_at TEXT NOT NULL
                );
            """)

    def session(self) -> sqlite3.Row | None:
        with self.connect() as db:
            return db.execute("SELECT * FROM session WHERE id=1").fetchone()

    def create_session(self, count: int, games: list[GameSpec]) -> None:
        stamp = now()
        with self.connect() as db:
            db.execute("INSERT INTO session (id, game_count, initialized_at) VALUES (1, ?, ?)", (count, stamp))
            for spec in games:
                db.execute("""INSERT INTO games (game_id,profile_id,round_number,fen,status,created_at,updated_at)
                    VALUES (?, ?, 1, ?, 'awaiting_initial_board', ?, ?)""",
                    (spec.game_id, spec.profile_id, STARTING_FEN, stamp, stamp))
                self.event(db, spec.game_id, 1, "session_initialized", {"profile_id": spec.profile_id})

    def event(self, db: sqlite3.Connection, game_id: str, rnd: int, event: str, detail: dict[str, Any]) -> None:
        db.execute("INSERT INTO events(game_id,round_number,event_type,detail,created_at) VALUES(?,?,?,?,?)",
                   (game_id, rnd, event, json.dumps(detail), now()))

    def game(self, game_id: str) -> sqlite3.Row:
        with self.connect() as db:
            row = db.execute("SELECT * FROM games WHERE game_id=?", (game_id,)).fetchone()
        if not row:
            raise HTTPException(404, "Unknown game_id")
        return row

    def games(self) -> list[sqlite3.Row]:
        with self.connect() as db:
            return db.execute("SELECT * FROM games ORDER BY game_id").fetchall()

    def move_rows(self, game_id: str, rnd: int) -> list[sqlite3.Row]:
        with self.connect() as db:
            return db.execute("SELECT * FROM moves WHERE game_id=? AND round_number=? ORDER BY ply", (game_id, rnd)).fetchall()

    def events(self, game_id: str) -> list[sqlite3.Row]:
        with self.connect() as db:
            return db.execute("SELECT * FROM events WHERE game_id=? ORDER BY id DESC LIMIT 100", (game_id,)).fetchall()

    def update(self, game_id: str, **fields: Any) -> None:
        fields["updated_at"] = now()
        cols = ", ".join(f"{key}=?" for key in fields)
        with self.connect() as db:
            db.execute(f"UPDATE games SET {cols} WHERE game_id=?", (*fields.values(), game_id))

    def record_move(self, game: sqlite3.Row, side: str, move: chess.Move, san: str, board: chess.Board, capture: str | None) -> None:
        with self.connect() as db:
            ply = db.execute("SELECT COUNT(*) FROM moves WHERE game_id=? AND round_number=?", (game["game_id"], game["round_number"])).fetchone()[0] + 1
            db.execute("""INSERT INTO moves(game_id,round_number,ply,side,uci,san,fen_after,is_capture,captured_piece,created_at)
                VALUES(?,?,?,?,?,?,?,?,?,?)""", (game["game_id"], game["round_number"], ply, side, move.uci(), san, board.fen(), bool(capture), capture, now()))

    def restart(self, game: sqlite3.Row, profile_id: str, reason: str | None) -> sqlite3.Row:
        stamp = now()
        with self.connect() as db:
            if game["status"] not in ("finished", "awaiting_initial_board"):
                db.execute("UPDATE games SET status='finished',result='aborted',result_type='restart',updated_at=? WHERE game_id=?", (stamp, game["game_id"]))
                self.event(db, game["game_id"], game["round_number"], "round_aborted", {"reason": reason})
            db.execute("""UPDATE games SET profile_id=?, round_number=?, fen=?, status='awaiting_initial_board',
                result=NULL,result_type=NULL,pending_move=NULL,pending_san=NULL,evaluation_cp=NULL,evaluation_label=NULL,last_error=NULL,updated_at=? WHERE game_id=?""",
                (profile_id, game["round_number"] + 1, STARTING_FEN, stamp, game["game_id"]))
            self.event(db, game["game_id"], game["round_number"] + 1, "round_restarted", {"profile_id": profile_id, "reason": reason})
        return self.game(game["game_id"])


def piece_name(piece: chess.Piece | None) -> str | None:
    return chess.piece_name(piece.piece_type) if piece else None


def terminal(board: chess.Board) -> tuple[str, str] | None:
    if board.is_checkmate():
        return ("robot_win" if board.turn == chess.BLACK else "player_win", "checkmate")
    if board.is_stalemate(): return ("draw", "stalemate")
    if board.is_insufficient_material(): return ("draw", "insufficient_material")
    if board.is_fivefold_repetition(): return ("draw", "fivefold_repetition")
    if board.is_seventyfive_moves(): return ("draw", "seventyfive_move_rule")
    return None


def response_game(store: Store, profiles: dict[str, Profile], game: sqlite3.Row) -> dict[str, Any]:
    moves = [dict(row) for row in store.move_rows(game["game_id"], game["round_number"])]
    out = dict(game)
    out["profile"] = profiles[game["profile_id"]].model_dump()
    out["moves"] = moves
    out["claimable_draw"] = False
    try:
        board = chess.Board(game["fen"])
        out["claimable_draw"] = board.can_claim_draw()
    except ValueError:
        pass
    return out


class MoveChooser:
    def __init__(self, binary: str | None) -> None:
        self.binary = binary

    def choose(self, board: chess.Board, profile: Profile) -> tuple[chess.Move, int | None]:
        legal = [m for m in board.legal_moves if not board.is_castling(m)]
        if not legal:
            raise RuntimeError("No non-castling legal robot move")
        if not self.binary:
            return legal[0], None
        try:
            with chess.engine.SimpleEngine.popen_uci(self.binary) as engine:
                infos = engine.analyse(board, chess.engine.Limit(time=profile.think_time_ms / 1000), multipv=min(5, len(legal)))
            candidates: list[tuple[chess.Move, int]] = []
            for info in infos:
                move = info["pv"][0]
                if board.is_castling(move):
                    continue
                score = info["score"].pov(chess.WHITE).score(mate_score=100000)
                candidates.append((move, score))
            if not candidates:
                return legal[0], None
            best = max(score for _, score in candidates)
            allowed = [(move, score) for move, score in candidates if score >= best - profile.candidate_window_cp]
            if profile.style == "tactical":
                captures = [item for item in allowed if board.is_capture(item[0]) or board.gives_check(item[0])]
                if captures: allowed = captures
            if profile.style == "cautious":
                allowed.sort(key=lambda item: item[1], reverse=True)
            return allowed[0]
        except Exception as exc:
            # A physical robot must remain usable when Stockfish is temporarily unavailable.
            return legal[0], None


def load_profiles(path: Path) -> dict[str, Profile]:
    raw = json.loads(path.read_text())
    profiles = {item["id"]: Profile.model_validate(item) for item in raw["profiles"]}
    if not profiles: raise ValueError("At least one profile is required")
    return profiles


def create_app(database_path: str | None = None, game_count: int | None = None, stockfish_path: str | None = None) -> FastAPI:
    db_path = database_path or os.getenv("CHESS_DATABASE", str(ROOT / "chess.db"))
    count = game_count or int(os.getenv("CHESS_GAME_COUNT", "5"))
    profiles = load_profiles(ROOT / "config" / "profiles.json")
    store = Store(db_path)
    broker = Broker()
    chooser = MoveChooser(stockfish_path or os.getenv("STOCKFISH_PATH"))

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        store.init()
        yield

    app = FastAPI(title="Chess Robot Digital Twin", lifespan=lifespan)
    app.mount("/static", StaticFiles(directory=ROOT / "static"), name="static")

    def require_active(game: sqlite3.Row, *states: str) -> None:
        if game["status"] not in states:
            raise HTTPException(409, {"error": "invalid_game_state", "status": game["status"], "expected": list(states)})

    def discrepancy(game: sqlite3.Row, reason: str, detail: dict[str, Any]) -> HTTPException:
        store.update(game["game_id"], last_error=reason)
        with store.connect() as db:
            store.event(db, game["game_id"], game["round_number"], "discrepancy", {"reason": reason, **detail})
        return HTTPException(409, {"error": "board_discrepancy", "reason": reason, "authoritative_fen": game["fen"], **detail})

    def select_robot_move(game: sqlite3.Row) -> dict[str, Any]:
        board = chess.Board(game["fen"])
        profile = profiles[game["profile_id"]]
        move, score = chooser.choose(board, profile)
        captured = board.piece_at(move.to_square) if not board.is_en_passant(move) else chess.Piece(chess.PAWN, chess.BLACK)
        is_capture = board.is_capture(move)
        san = board.san(move)
        store.update(game["game_id"], pending_move=move.uci(), pending_san=san, evaluation_cp=score,
                     evaluation_label="winning" if score is not None and score >= 100 else "losing" if score is not None and score <= -100 else "neutral")
        return {"uci": move.uci(), "san": san, "from": chess.square_name(move.from_square), "to": chess.square_name(move.to_square),
                "captures_piece": is_capture, "captured_piece": piece_name(captured), "gives_check": board.gives_check(move),
                "evaluation_cp": score, "evaluation_label": "winning" if score is not None and score >= 100 else "losing" if score is not None and score <= -100 else "neutral"}

    @app.get("/")
    async def dashboard() -> FileResponse:
        return FileResponse(ROOT / "static" / "index.html")

    @app.get("/v1/profiles")
    async def get_profiles() -> list[dict[str, Any]]:
        return [p.model_dump() for p in profiles.values()]

    @app.post("/v1/session")
    async def start_session(request: SessionRequest) -> dict[str, Any]:
        if store.session(): raise HTTPException(409, "A session already exists; restart the service/database for a new one")
        if len(request.games) != count: raise HTTPException(422, f"Exactly {count} games are required")
        if len({g.game_id for g in request.games}) != count: raise HTTPException(422, "game_id values must be unique")
        unknown = [g.profile_id for g in request.games if g.profile_id not in profiles]
        if unknown: raise HTTPException(422, {"error": "unknown_profile", "profile_ids": unknown})
        store.create_session(count, request.games)
        games = [response_game(store, profiles, store.game(g.game_id)) for g in request.games]
        await broker.publish("session_initialized", {"games": games})
        return {"game_count": count, "games": games}

    @app.get("/v1/session")
    async def get_session() -> dict[str, Any]:
        session = store.session()
        if not session: return {"initialized": False, "game_count": count, "games": []}
        return {"initialized": True, "game_count": session["game_count"], "games": [response_game(store, profiles, g) for g in store.games()]}

    @app.get("/v1/games/{game_id}")
    async def get_game(game_id: str) -> dict[str, Any]:
        return response_game(store, profiles, store.game(game_id))

    @app.get("/v1/games/{game_id}/events")
    async def game_events(game_id: str) -> list[dict[str, Any]]:
        store.game(game_id)
        return [dict(row) for row in store.events(game_id)]

    @app.post("/v1/games/{game_id}/initial-board-confirmation")
    async def confirm_initial_board(game_id: str, request: InitialBoardRequest) -> dict[str, Any]:
        game = store.game(game_id); require_active(game, "awaiting_initial_board")
        if request.observed_fen != STARTING_FEN:
            raise discrepancy(game, "initial_board_not_standard", {"observed_fen": request.observed_fen})
        store.update(game_id, status="awaiting_robot_completion", last_error=None)
        game = store.game(game_id)
        move = select_robot_move(game)
        payload = response_game(store, profiles, store.game(game_id)) | {"robot_move": move}
        await broker.publish("robot_move_selected", payload)
        return payload

    @app.post("/v1/games/{game_id}/robot-move-completion")
    async def robot_complete(game_id: str, request: RobotCompletionRequest) -> dict[str, Any]:
        game = store.game(game_id)
        if game["status"] == "awaiting_player_move" and not game["pending_move"]:
            return response_game(store, profiles, game)  # idempotent success
        require_active(game, "awaiting_robot_completion")
        if not request.completed: raise HTTPException(422, "completed must be true")
        board = chess.Board(game["fen"]); move = chess.Move.from_uci(game["pending_move"])
        expected = board.copy(); san = board.san(move); capture = piece_name(board.piece_at(move.to_square)) if board.is_capture(move) else None; expected.push(move)
        if request.observed_fen is not None and request.observed_fen != expected.fen():
            raise discrepancy(game, "robot_move_not_observed", {"observed_fen": request.observed_fen, "expected_fen": expected.fen()})
        store.record_move(game, "robot", move, san, expected, capture)
        outcome = terminal(expected)
        fields: dict[str, Any] = {"fen": expected.fen(), "pending_move": None, "pending_san": None, "last_error": None}
        if outcome: fields.update(status="finished", result=outcome[0], result_type=outcome[1])
        else: fields["status"] = "awaiting_player_move"
        store.update(game_id, **fields)
        payload = response_game(store, profiles, store.game(game_id))
        await broker.publish("robot_move_completed", payload)
        return payload

    @app.post("/v1/games/{game_id}/player-move")
    async def player_move(game_id: str, request: PlayerMoveRequest) -> dict[str, Any]:
        game = store.game(game_id); require_active(game, "awaiting_player_move")
        board = chess.Board(game["fen"])
        try: move = chess.Move.from_uci(request.move)
        except ValueError: raise discrepancy(game, "malformed_move", {"move": request.move})
        if move not in board.legal_moves: raise discrepancy(game, "illegal_player_move", {"move": request.move})
        expected = board.copy(); san = board.san(move); capture = piece_name(board.piece_at(move.to_square)) if board.is_capture(move) else None; expected.push(move)
        if request.observed_fen != expected.fen():
            raise discrepancy(game, "player_move_and_board_do_not_match", {"move": request.move, "observed_fen": request.observed_fen, "expected_fen": expected.fen()})
        store.record_move(game, "player", move, san, expected, capture)
        outcome = terminal(expected)
        fields: dict[str, Any] = {"fen": expected.fen(), "last_error": None}
        if outcome:
            fields.update(status="finished", result=outcome[0], result_type=outcome[1]); store.update(game_id, **fields)
            payload = response_game(store, profiles, store.game(game_id)); await broker.publish("game_finished", payload); return payload
        fields["status"] = "awaiting_robot_completion"; store.update(game_id, **fields)
        move_data = select_robot_move(store.game(game_id))
        payload = response_game(store, profiles, store.game(game_id)) | {"robot_move": move_data}
        await broker.publish("robot_move_selected", payload)
        return payload

    @app.post("/v1/games/{game_id}/forfeit")
    async def forfeit(game_id: str, request: ForfeitRequest) -> dict[str, Any]:
        game = store.game(game_id); require_active(game, "awaiting_initial_board", "awaiting_robot_completion", "awaiting_player_move")
        result = "player_win" if request.forfeiting_side == "robot" else "robot_win"
        store.update(game_id, status="finished", result=result, result_type="forfeit", pending_move=None, pending_san=None)
        with store.connect() as db: store.event(db, game_id, game["round_number"], "forfeit", request.model_dump())
        payload = response_game(store, profiles, store.game(game_id)); await broker.publish("game_forfeited", payload); return payload

    @app.post("/v1/games/{game_id}/restart")
    async def restart(game_id: str, request: RestartRequest) -> dict[str, Any]:
        game = store.game(game_id); profile_id = request.profile_id or game["profile_id"]
        if profile_id not in profiles: raise HTTPException(422, "Unknown profile_id")
        new_game = store.restart(game, profile_id, request.reason)
        payload = response_game(store, profiles, new_game); await broker.publish("game_restarted", payload); return payload

    @app.websocket("/v1/events")
    async def events(ws: WebSocket) -> None:
        await broker.connect(ws)
        try:
            while True: await ws.receive_text()
        except WebSocketDisconnect:
            broker.disconnect(ws)

    return app


app = create_app()
