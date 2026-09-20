--[==[badge-app
slug=htn_doomcrawl
name=Doom Crawl
icon=DM
api=2
heap_kb=48
wake_lock=1
]==]

-- A first-person dungeon crawler in the spirit of Doom, built for this
-- badge's widget-based UI. There is no raw pixel/canvas API, so this casts
-- a small fixed number of rays (one per reused "wall strip" box) across a
-- 60-degree field of view, shades each strip by distance/side, and traces
-- two `line` widgets along the strip tops/bottoms -- those connecting
-- segments are what actually render as diagonal edges between strips of
-- different height, faking real perspective without a per-pixel canvas.

local MAP = {
  "############",
  "#..........#",
  "#.####.###.#",
  "#.#..#.#...#",
  "#.#.##.#.###",
  "#...#..#...#",
  "###.#.##.#.#",
  "#...#..#.#.#",
  "#.###.##.#.#",
  "#.....#....#",
  "#.####.###.#",
  "############",
}
local MAP_W, MAP_H = 12, 12

local TWO_PI = 2 * math.pi
local N_COLS = 9
local HALF_FOV = math.rad(30)
local COS_HALF_FOV = math.cos(HALF_FOV)
local SIN_HALF_FOV = math.sin(HALF_FOV)
local TURN_STEP = math.rad(15)
local MOVE_STEP = 0.3
local BUMP_R = 0.5
local ATTACK_CONE_COS = math.cos(math.rad(12))
local ATTACK_RANGE = 6

-- Screen geometry: a 280px-wide, 160px-tall corridor viewport centered at
-- (160, 120), below the title and above the HUD text.
local CORRIDOR_LEFT = 20
local CORRIDOR_W = 280
local CENTER_X = 160
local MID_Y = 120
local MAX_H = 160
local SCALE = 110

local px, py, angle
local hp, max_hp
local monsters
local kills
local msg
local dead = false

-- Widgets, created once in on_enter and only ever repositioned/recolored.
local ceiling, floor
local wall_boxes
local roof_line, floor_line
local monster_box
local monster_eye_l, monster_eye_r
local gun_body, gun_barrel, gun_sight, muzzle_flash
local hp_label, kills_label, msg_label, hint_label

-- One-shot flag: render_view() shows the muzzle flash for exactly the one
-- render right after firing, then clears it (there is no on_tick to time
-- a real fade against).
local just_fired = false

local function is_wall(x, y)
  if x < 0 or x >= MAP_W or y < 0 or y >= MAP_H then return true end
  return string.sub(MAP[y + 1], x + 1, x + 1) == "#"
end

-- Grid-DDA raycast from (x0,y0) toward angle ang. Returns Euclidean
-- distance to the hit wall and which grid side was crossed (0 = vertical
-- line, 1 = horizontal line), used for cheap left/right wall shading.
local function cast_ray(x0, y0, ang)
  local dx, dy = math.cos(ang), math.sin(ang)
  local map_x, map_y = math.floor(x0), math.floor(y0)
  local delta_x = (dx == 0) and 1e30 or math.abs(1 / dx)
  local delta_y = (dy == 0) and 1e30 or math.abs(1 / dy)
  local step_x, side_x
  local step_y, side_y
  if dx < 0 then
    step_x = -1
    side_x = (x0 - map_x) * delta_x
  else
    step_x = 1
    side_x = (map_x + 1 - x0) * delta_x
  end
  if dy < 0 then
    step_y = -1
    side_y = (y0 - map_y) * delta_y
  else
    step_y = 1
    side_y = (map_y + 1 - y0) * delta_y
  end

  local side = 0
  local hit = false
  local guard = 0
  while not hit and guard < 48 do
    guard = guard + 1
    if side_x < side_y then
      side_x = side_x + delta_x
      map_x = map_x + step_x
      side = 0
    else
      side_y = side_y + delta_y
      map_y = map_y + step_y
      side = 1
    end
    if map_x < 0 or map_x >= MAP_W or map_y < 0 or map_y >= MAP_H then
      hit = true
    elseif is_wall(map_x, map_y) then
      hit = true
    end
  end

  local dist = (side == 0) and (side_x - delta_x) or (side_y - delta_y)
  if dist < 0.05 then dist = 0.05 end
  return dist, side
end

local function monster_near(x, y, radius)
  local best, best_d2 = nil, radius * radius
  for i, m in ipairs(monsters) do
    if m.alive then
      local ddx, ddy = x - m.x, y - m.y
      local d2 = ddx * ddx + ddy * ddy
      if d2 <= best_d2 then
        best, best_d2 = i, d2
      end
    end
  end
  return best
end

local function reset_game()
  px, py, angle = 1.5, 1.5, math.pi / 2 -- start near the entrance, facing south
  hp, max_hp = 6, 6
  kills = 0
  dead = false
  monsters = {
    { x = 8.5, y = 1.5, hp = 2, alive = true },
    { x = 9.5, y = 3.5, hp = 2, alive = true },
    { x = 2.5, y = 5.5, hp = 2, alive = true },
    { x = 8.5, y = 7.5, hp = 2, alive = true },
    { x = 5.5, y = 9.5, hp = 2, alive = true },
  }
  msg = "A maze. Something growls nearby."
end

local function update_hud()
  hp_label:set_text("HP " .. hp .. "/" .. max_hp)
  kills_label:set_text("Kills " .. kills .. "/" .. #monsters)
  msg_label:set_text(msg)
end

local function set_leds()
  local hp_leds = math.max(0, math.min(3, math.ceil(hp / 2)))
  local alive_count = 0
  for _, m in ipairs(monsters) do
    if m.alive then alive_count = alive_count + 1 end
  end
  local mo_leds = math.max(0, math.min(3, math.ceil(alive_count / 2)))
  local left = { 1, 6, 5 }
  local right = { 2, 3, 4 }
  for i = 1, 3 do
    if i <= hp_leds then badge.led.set(left[i], 200, 0, 0) else badge.led.set(left[i], 0, 0, 0) end
    if i <= mo_leds then badge.led.set(right[i], 0, 160, 0) else badge.led.set(right[i], 0, 0, 0) end
  end
  badge.led.show()
end

-- One-shot color override, shown until the next action recomputes set_leds().
local function flash_leds(r, g, b)
  badge.led.set_all(r, g, b)
  badge.led.show()
end

local function wall_color(perp, side)
  local r, g, b = 0x6a, 0x72, 0x7c
  if side == 1 then
    r, g, b = r * 0.72, g * 0.72, b * 0.72
  end
  local fog = 1 - math.min(1, perp / 7)
  local k = 0.25 + 0.75 * fog
  r = math.floor(r * k)
  g = math.floor(g * k)
  b = math.floor(b * k)
  return r * 0x10000 + g * 0x100 + b
end

-- Axis-separated collision so movement along a non-axis-aligned angle
-- slides smoothly along walls instead of stopping dead at any contact.
local function try_move(nx, ny)
  if not is_wall(math.floor(nx), math.floor(py)) then px = nx end
  if not is_wall(math.floor(px), math.floor(ny)) then py = ny end
end

local function render_view()
  local fx, fy = math.cos(angle), math.sin(angle)
  local col_w = CORRIDOR_W / N_COLS
  local col_dist = {}
  local roof_pts, floor_pts = {}, {}

  for i = 0, N_COLS - 1 do
    local t = (i - (N_COLS - 1) / 2) / ((N_COLS - 1) / 2)
    local ray_ang = angle + t * HALF_FOV
    local dist, side = cast_ray(px, py, ray_ang)
    local perp = dist * math.cos(ray_ang - angle)
    if perp < 0.1 then perp = 0.1 end
    col_dist[i + 1] = perp

    local height = SCALE / perp
    if height > MAX_H then height = MAX_H end
    local x0 = math.floor(CORRIDOR_LEFT + i * col_w)
    local x1 = math.floor(CORRIDOR_LEFT + (i + 1) * col_w) + 1
    local y0 = math.floor(MID_Y - height / 2)
    local y1 = math.floor(MID_Y + height / 2)

    local box = wall_boxes[i + 1]
    box:set_pos(x0, y0)
    box:set_size(x1 - x0, y1 - y0)
    box:style({ bg_color = wall_color(perp, side) })

    local cx = math.floor((x0 + x1) / 2)
    roof_pts[#roof_pts + 1] = { cx, y0 }
    floor_pts[#floor_pts + 1] = { cx, y1 }
  end

  roof_line:set_points(roof_pts)
  floor_line:set_points(floor_pts)

  local near_i, near_d, near_sx = nil, 1e9, 0
  for i, m in ipairs(monsters) do
    if m.alive then
      local ddx, ddy = m.x - px, m.y - py
      local d = math.sqrt(ddx * ddx + ddy * ddy)
      if d > 0.05 and d < near_d then
        local ux, uy = ddx / d, ddy / d
        local dot = fx * ux + fy * uy
        if dot > COS_HALF_FOV then
          -- cross ~= sin(angle-to-target) for unit vectors; a fine small-
          -- angle stand-in for atan2, which this Lua runtime may not have.
          local cross = fx * uy - fy * ux
          local sx = CENTER_X + (cross / SIN_HALF_FOV) * (CORRIDOR_W / 2)
          local col_i = math.floor((sx - CORRIDOR_LEFT) / col_w)
          if col_i < 0 then col_i = 0 end
          if col_i > N_COLS - 1 then col_i = N_COLS - 1 end
          if d < col_dist[col_i + 1] + 0.15 then
            near_i, near_d, near_sx = i, d, sx
          end
        end
      end
    end
  end

  if near_i then
    local size = SCALE * 0.6 / near_d
    if size > 70 then size = 70 end
    if size < 10 then size = 10 end
    local mx = math.floor(near_sx - size / 2)
    local my = math.floor(MID_Y - size * 0.6)
    local mw, mh = math.floor(size), math.floor(size * 1.2)

    monster_box:hidden(false)
    monster_box:set_pos(mx, my)
    monster_box:set_size(mw, mh)

    -- Glowing eyes give the blob a "creature in the dark" read without
    -- needing real sprite art, which this API has no way to draw at runtime.
    local eye_size = math.max(3, math.floor(size * 0.16))
    local eye_y = my + math.floor(size * 0.22)
    monster_eye_l:hidden(false)
    monster_eye_l:set_size(eye_size, eye_size)
    monster_eye_l:set_pos(mx + math.floor(size * 0.2), eye_y)
    monster_eye_r:hidden(false)
    monster_eye_r:set_size(eye_size, eye_size)
    monster_eye_r:set_pos(mx + math.floor(size * 0.66), eye_y)
  else
    monster_box:hidden(true)
    monster_eye_l:hidden(true)
    monster_eye_r:hidden(true)
  end

  muzzle_flash:hidden(not just_fired)
  just_fired = false

  update_hud()
  set_leds()
end

function on_enter(root)
  local title = badge.ui.label(root, "DOOM CRAWL")
  title:style({ text_font = 20 })
  title:align("top_mid", 0, 6)

  ceiling = badge.ui.box(root, 320, MID_Y - 40)
  ceiling:set_pos(0, 40)
  ceiling:style({ bg_color = 0x161a20, radius = 0, border_width = 0 })

  floor = badge.ui.box(root, 320, MID_Y - 40)
  floor:set_pos(0, MID_Y)
  floor:style({ bg_color = 0x241b14, radius = 0, border_width = 0 })

  wall_boxes = {}
  for i = 1, N_COLS do
    local b = badge.ui.box(root, 10, 10)
    b:style({ radius = 0, border_width = 0 })
    wall_boxes[i] = b
  end

  roof_line = badge.ui.line(root, { { 0, 0 }, { 1, 1 } })
  roof_line:style({ line_color = 0xd8dce2, line_width = 2 })

  floor_line = badge.ui.line(root, { { 0, 0 }, { 1, 1 } })
  floor_line:style({ line_color = 0x3a3228, line_width = 2 })

  monster_box = badge.ui.box(root, 40, 54)
  monster_box:style({ bg_color = 0xcc2222, radius = 6, border_width = 0 })

  monster_eye_l = badge.ui.box(root, 6, 6)
  monster_eye_l:style({ bg_color = 0xffe066, radius = 3, border_width = 0 })

  monster_eye_r = badge.ui.box(root, 6, 6)
  monster_eye_r:style({ bg_color = 0xffe066, radius = 3, border_width = 0 })

  -- First-person gun overlay, anchored bottom-center. Created after the
  -- monster so it always draws in front of walls/creatures, like a real
  -- FPS weapon. Static -- only muzzle_flash's visibility changes per frame.
  gun_body = badge.ui.box(root, 78, 30)
  gun_body:set_pos(121, 170)
  gun_body:style({ bg_color = 0x2a2d33, radius = 4, border_width = 0 })

  gun_barrel = badge.ui.box(root, 16, 46)
  gun_barrel:set_pos(152, 150)
  gun_barrel:style({ bg_color = 0x33363d, radius = 2, border_width = 0 })

  gun_sight = badge.ui.box(root, 8, 6)
  gun_sight:set_pos(156, 146)
  gun_sight:style({ bg_color = 0x14151a, radius = 1, border_width = 0 })

  muzzle_flash = badge.ui.box(root, 26, 20)
  muzzle_flash:set_pos(147, 128)
  muzzle_flash:style({ bg_color = 0xfff2b0, radius = 4, border_width = 0 })
  muzzle_flash:hidden(true)

  hp_label = badge.ui.label(root, "")
  hp_label:style({ text_font = 14 })
  hp_label:align("top_left", 8, 6)

  kills_label = badge.ui.label(root, "")
  kills_label:style({ text_font = 14 })
  kills_label:align("top_right", -8, 6)

  msg_label = badge.ui.label(root, "")
  msg_label:style({ text_font = 16, text_align = "center" })
  msg_label:align("bottom_mid", 0, -40)

  hint_label = badge.ui.label(root,
    "Up/Down move   Left/Right look\nA attack   Start reset")
  hint_label:style({ text_font = 14, text_align = "center" })
  hint_label:align("bottom_mid", 0, -10)

  reset_game()
  render_view()
end

function on_button(button, kind)
  if kind ~= badge.input.KIND.PRESSED then return end
  local B = badge.input.BUTTON

  if button == B.START then
    reset_game()
    render_view()
    return
  end

  if dead then return end

  if button == B.LEFT then
    angle = (angle - TURN_STEP) % TWO_PI
    msg = "You look left."
  elseif button == B.RIGHT then
    angle = (angle + TURN_STEP) % TWO_PI
    msg = "You look right."
  elseif button == B.UP or button == B.DOWN then
    local sign = (button == B.UP) and 1 or -1
    local nx = px + math.cos(angle) * MOVE_STEP * sign
    local ny = py + math.sin(angle) * MOVE_STEP * sign
    local bi = monster_near(nx, ny, BUMP_R)
    if bi then
      hp = hp - 2
      flash_leds(200, 0, 0)
      if hp <= 0 then
        hp = 0
        dead = true
        msg = "The creature overwhelms you. Press Start to respawn."
      else
        msg = "A creature blocks your path and claws you! -2 HP"
      end
    else
      local ox, oy = px, py
      try_move(nx, ny)
      if px == ox and py == oy then
        msg = "A wall blocks the way."
      elseif sign > 0 then
        msg = "You edge forward."
      else
        msg = "You step back."
      end
    end
  elseif button == B.A then
    just_fired = true
    local fx, fy = math.cos(angle), math.sin(angle)
    local wall_dist = cast_ray(px, py, angle)
    local best, best_d = nil, 1e9
    for i, m in ipairs(monsters) do
      if m.alive then
        local ddx, ddy = m.x - px, m.y - py
        local d = math.sqrt(ddx * ddx + ddy * ddy)
        if d > 0.05 and d < ATTACK_RANGE and d < wall_dist + 0.3 and d < best_d then
          local dot = (fx * ddx + fy * ddy) / d
          if dot > ATTACK_CONE_COS then
            best, best_d = i, d
          end
        end
      end
    end
    if best then
      local m = monsters[best]
      m.hp = m.hp - 1
      flash_leds(255, 255, 255)
      if m.hp <= 0 then
        m.alive = false
        kills = kills + 1
        if kills >= #monsters then
          msg = "All creatures defeated! Press Start to play again."
        else
          msg = "You defeated the creature!"
        end
      else
        msg = "You strike the creature!"
      end
    else
      msg = "You attack, but there is nothing there."
    end
  else
    return
  end

  render_view()
end

function on_exit()
  badge.led.clear()
  badge.led.show()
end
