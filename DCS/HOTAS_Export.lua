-- =========================================================
-- HOTAS Haptic Telemetry Bridge v4
-- Generic (FC3 + Full Fidelity), chained, text file output
-- Line: TIME; AC; STATE; G; AOA; TD; SINK; GUN; MSL; PL;
--       PITCH; BANK; HDG; ALT; IAS; MACH; VS
-- SINK is latched: sink rate (m/s) of the LAST touchdown. VS is live.
-- =========================================================

local hotas_path = lfs.writedir() .. "Temp\\hotas_telemetry.txt"

-- Tunables
local WRITE_PERIOD   = 1 / 30   -- s between file writes (events are polled every frame)
local GND_OFFSET_DEF = 3.0      -- m, radar-alt reading when parked (auto-calibrated)
local TAXI_MIN_SPEED = 0.5      -- m/s, below this on ground = PARKED
local ROLL_MIN_SPEED = 20       -- m/s (~39 kt), above this on ground = ROLL
local AOA_MIN_SPEED  = 15       -- m/s, AoA is meaningless below this
local MIN_AIR_TIME   = 1.0      -- s airborne required to count a touchdown
local HOLD_TD        = 1.0      -- s, event flags stay at 1 for this long
local HOLD_GUN       = 0.25
local HOLD_MSL       = 1.0
local G0             = 9.80665

-- Chain any previously defined export callbacks (SRS, Tacview, DCS-BIOS, ...)
local prev_start = LuaExportStart
local prev_after = LuaExportAfterNextFrame
local prev_stop  = LuaExportStop

local S = {}

local function reset_state(name)
    S.aircraft    = name
    S.on_ground   = nil
    S.calibrated  = false
    S.gnd_offset  = GND_OFFSET_DEF
    S.air_since   = 0
    S.prev_vv     = 0
    S.sink        = 0
    S.td_until    = 0
    S.gun_until   = 0
    S.msl_until   = 0
    S.prev_shells = nil
    S.prev_counts = nil
    S.pv          = nil     -- previous velocity sample (G fallback)
    S.g_lp        = 1.0
end
reset_state("NONE")
S.last_write = 0

-- Call an API function safely; returns nil if it is missing or errors out
local function try(fn, ...)
    if type(fn) ~= "function" then return nil end
    local ok, res = pcall(fn, ...)
    if ok then return res end
    return nil
end

-- Proven method: reopen with "w" (truncate), write, close. No seek(), no "r+".
local function write_line(line)
    line = (tostring(line):gsub("[\r\n]+", " "))
    local f = io.open(hotas_path, "w")
    if f then
        f:write(line, "\n")
        f:close()
    end
end

-- World vector (DCS: x=north, y=up, z=east -> NED) to body frame (forward, right, down)
local function ned_to_body(sd, n, e, d)
    local cp, sp = math.cos(sd.Heading), math.sin(sd.Heading)
    local ct, st = math.cos(sd.Pitch),   math.sin(sd.Pitch)
    local cb, sb = math.cos(sd.Bank),    math.sin(sd.Bank)

    local x1 =  n * cp + e * sp
    local y1 = -n * sp + e * cp
    local u  =  x1 * ct - d * st
    local z2 =  x1 * st + d * ct
    local v  =  y1 * cb + z2 * sb
    local w  = -y1 * sb + z2 * cb
    return u, v, w
end

-- AoA from body-frame velocity (LoGetAngleOfAttack is unreliable on some modules)
local function compute_aoa(sd, vel)
    if not (sd and vel and sd.Heading and sd.Pitch and sd.Bank) then return 0 end
    local vn, ve, vd = vel.x, vel.z, -vel.y
    if math.sqrt(vn * vn + ve * ve + vd * vd) < AOA_MIN_SPEED then return 0 end
    local u, _, w = ned_to_body(sd, vn, ve, vd)
    return math.deg(math.atan2(w, u))
end

-- G fallback for modules where LoGetAccelerationUnits is missing or all zeros:
-- differentiate the velocity vector, remove gravity, project onto the body Z axis
local function g_fallback(sd, vel, t)
    local result = nil
    if sd and vel and sd.Heading and sd.Pitch and sd.Bank then
        local pv = S.pv
        if pv and t > pv.t then
            local dt = t - pv.t
            if dt > 0.001 and dt < 0.5 then
                local an = (vel.x - pv.n) / dt
                local ae = (vel.z - pv.e) / dt
                local ad = (-vel.y - pv.d) / dt
                local _, _, fz = ned_to_body(sd, an, ae, ad - G0)   -- specific force
                result = -fz / G0
            end
        end
        S.pv = { n = vel.x, e = vel.z, d = -vel.y, t = t }
    else
        S.pv = nil
    end
    return result
end

local function read_g(sd, vel, t)
    local fb  = g_fallback(sd, vel, t)   -- always runs to keep the sample fresh
    local acc = try(LoGetAccelerationUnits)
    if type(acc) == "table" and type(acc.y) == "number"
       and not (acc.x == 0 and acc.y == 0 and acc.z == 0) then
        return acc.y
    end
    if fb then
        S.g_lp = S.g_lp + 0.3 * (fb - S.g_lp)   -- low-pass, fallback is noisy
        return S.g_lp
    end
    return 1.0
end

-- Weight-on-wheels is module specific: infer ground contact from radar altitude
local function update_ground(now, agl, vv, gs)
    -- Calibrate the radar-alt offset once, while stationary near the ground
    if not S.calibrated and gs < 1 and math.abs(vv) < 0.2 and agl < 15 then
        S.gnd_offset = agl
        S.calibrated = true
    end

    local was = S.on_ground
    if was == nil then
        S.on_ground = agl < S.gnd_offset + 1.5
        if not S.on_ground then S.air_since = now end
    elseif was and agl > S.gnd_offset + 2.0 then
        S.on_ground = false                          -- takeoff
        S.air_since = now
    elseif (not was) and agl < S.gnd_offset + 0.7 then
        S.on_ground = true                           -- touchdown
        if now - S.air_since > MIN_AIR_TIME then
            S.sink     = math.max(0, -S.prev_vv)     -- last airborne vertical speed
            S.td_until = now + HOLD_TD
        end
    end

    if not S.on_ground then S.prev_vv = vv end
end

-- Returns true if payload info is available for this module
local function poll_weapons(now)
    local p = try(LoGetPayloadInfo)
    if type(p) ~= "table" then return false end

    -- Gun: cannon shell count dropping
    local shells = p.Cannon and p.Cannon.shells
    if type(shells) == "number" then
        if S.prev_shells and shells < S.prev_shells then S.gun_until = now + HOLD_GUN end
        S.prev_shells = shells
    end

    -- Stores: any station count dropping (missile / rocket / bomb release)
    local counts = {}
    if type(p.Stations) == "table" then
        for i, st in pairs(p.Stations) do
            local c = (type(st) == "table" and tonumber(st.count)) or 0
            counts[i] = c
            if S.prev_counts and S.prev_counts[i] and c < S.prev_counts[i] then
                S.msl_until = now + HOLD_MSL
            end
        end
    end
    S.prev_counts = counts
    return true
end

local function flight_state(gs)
    if not S.on_ground then return "AIRBORNE" end
    if gs < TAXI_MIN_SPEED then return "PARKED" end
    if gs > ROLL_MIN_SPEED then return "ROLL" end
    return "TAXI"
end

function LuaExportStart()
    if prev_start then prev_start() end
    write_line("TIME:0.0; AC:NONE; STATE:INIT")
end

function LuaExportAfterNextFrame()
    if prev_after then prev_after() end

    local ok, err = pcall(function()
        local now = os.clock()
        local t   = try(LoGetModelTime) or 0
        local sd  = try(LoGetSelfData)
        local line

        if type(sd) ~= "table" or not sd.Name then
            if S.aircraft ~= "NONE" then reset_state("NONE") end
            line = string.format("TIME:%.1f; AC:NONE; STATE:NO_AIRCRAFT", t)
        else
            if sd.Name ~= S.aircraft then reset_state(sd.Name) end

            local agl = try(LoGetAltitudeAboveGroundLevel) or 0   -- m
            local vv  = try(LoGetVerticalVelocity) or 0           -- m/s
            local vel = try(LoGetVectorVelocity)                  -- m/s, world frame
            local gs  = vel and math.sqrt(vel.x * vel.x + vel.z * vel.z) or 0

            local g   = read_g(sd, vel, t)
            local aoa = compute_aoa(sd, vel)

            -- Event detection runs every frame, even when the write is throttled
            update_ground(now, agl, vv, gs)
            local pl = poll_weapons(now)

            -- Attitude, heading and air data for the horizon / instruments
            local pitch = math.deg(sd.Pitch or 0)
            local bank  = math.deg(sd.Bank or 0)
            local hdg   = math.deg(sd.Heading or 0) % 360
            local alt   = try(LoGetAltitudeAboveSeaLevel) or 0              -- m
            local ias   = (try(LoGetIndicatedAirSpeed) or 0) * 3.6          -- km/h
            local mach  = try(LoGetMachNumber) or 0

            line = string.format(
                "TIME:%.1f; AC:%s; STATE:%s; G:%.2f; AOA:%.2f; TD:%d; SINK:%.1f; GUN:%d; MSL:%d; PL:%d; " ..
                "PITCH:%.1f; BANK:%.1f; HDG:%.0f; ALT:%.0f; IAS:%.0f; MACH:%.2f; VS:%.1f",
                t, S.aircraft, flight_state(gs), g, aoa,
                now < S.td_until  and 1 or 0, S.sink,
                now < S.gun_until and 1 or 0,
                now < S.msl_until and 1 or 0,
                pl and 1 or 0,
                pitch, bank, hdg, alt, ias, mach, vv)   -- VS: live vertical speed, m/s, + = climbing
        end

        if now - S.last_write >= WRITE_PERIOD then
            S.last_write = now
            write_line(line)
        end
    end)

    if not ok then write_line("ERR:" .. tostring(err)) end
end

function LuaExportStop()
    if prev_stop then prev_stop() end
    write_line("STATE:OFFLINE")
end