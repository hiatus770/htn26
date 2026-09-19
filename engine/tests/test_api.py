import pytest
from httpx import ASGITransport, AsyncClient

from engine.app import STARTING_FEN, create_app


@pytest.fixture
def anyio_backend():
    return "asyncio"


async def start(c):
    result = await c.post("/v1/session", json={"games": [
        {"game_id": "board-1", "profile_id": "balanced"},
        {"game_id": "board-2", "profile_id": "tactical"},
    ]})
    assert result.status_code == 200


@pytest.mark.anyio
async def test_session_requires_fixed_board_count(tmp_path):
    app = create_app(database_path=str(tmp_path / "chess.db"), game_count=2)
    async with app.router.lifespan_context(app), AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        response = await c.post("/v1/session", json={"games": [{"game_id": "only", "profile_id": "balanced"}]})
        assert response.status_code == 422


@pytest.mark.anyio
async def test_robot_opening_then_player_move_and_forfeit(tmp_path):
    app = create_app(database_path=str(tmp_path / "chess.db"), game_count=2)
    async with app.router.lifespan_context(app), AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        await start(c)
        opening = await c.post("/v1/games/board-1/initial-board-confirmation", json={"observed_fen": STARTING_FEN})
        assert opening.status_code == 200
        assert opening.json()["robot_move"]["uci"]
        game = (await c.get("/v1/games/board-1")).json()
        # Complete the planned move using the service's own authoritative transition.
        import chess
        board = chess.Board(game["fen"]); board.push_uci(game["pending_move"])
        done = await c.post("/v1/games/board-1/robot-move-completion", json={"observed_fen": board.fen()})
        assert done.status_code == 200
        assert done.json()["status"] == "awaiting_player_move"
        forfeited = await c.post("/v1/games/board-1/forfeit", json={"forfeiting_side": "player"})
        assert forfeited.status_code == 200
        assert forfeited.json()["result"] == "robot_win"
        assert forfeited.json()["result_type"] == "forfeit"


@pytest.mark.anyio
async def test_mismatched_physical_board_is_non_mutating(tmp_path):
    app = create_app(database_path=str(tmp_path / "chess.db"), game_count=2)
    async with app.router.lifespan_context(app), AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        await start(c)
        response = await c.post("/v1/games/board-1/initial-board-confirmation", json={"observed_fen": "not a fen"})
        assert response.status_code == 409
        assert (await c.get("/v1/games/board-1")).json()["status"] == "awaiting_initial_board"


@pytest.mark.anyio
async def test_restart_preserves_slot_and_advances_round(tmp_path):
    app = create_app(database_path=str(tmp_path / "chess.db"), game_count=2)
    async with app.router.lifespan_context(app), AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        await start(c)
        response = await c.post("/v1/games/board-1/restart", json={"profile_id": "magnus-carlbot"})
        assert response.status_code == 200
        body = response.json()
        assert body["round_number"] == 2
        assert body["profile_id"] == "magnus-carlbot"
        assert body["status"] == "awaiting_initial_board"
