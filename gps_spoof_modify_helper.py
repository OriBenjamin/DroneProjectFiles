# spoof_helpers.py
import math, time, random

# convert meters -> degree offsets (approx)
def meters_to_deg_offsets(lat_deg, north_m=0.0, east_m=0.0, up_m=0.0):
    meters_per_deg_lat = 111_320.0
    lat_rad = math.radians(lat_deg)
    meters_per_deg_lon = max(1e-6, 111_320.0 * math.cos(lat_rad))
    return north_m / meters_per_deg_lat, east_m / meters_per_deg_lon, up_m

# 1) smooth random walk drift (stateful)
_rw_state = {"north_m": 0.0, "east_m": 0.0, "up_m": 0.0, "t": None}
def spoof_random_walk(lat, lon, alt, max_step_m_per_s=0.2, decay=0.98, max_abs_m=30.0):
    now = time.time()
    dt = 0.1 if _rw_state["t"] is None else max(1e-3, now - _rw_state["t"])
    _rw_state["t"] = now
    step = max_step_m_per_s * dt
    _rw_state["north_m"] = max(-max_abs_m, min(max_abs_m, _rw_state["north_m"]*decay + random.uniform(-1,1)*step))
    _rw_state["east_m"]  = max(-max_abs_m, min(max_abs_m, _rw_state["east_m"]*decay + random.uniform(-1,1)*step))
    _rw_state["up_m"]    = max(-max_abs_m, min(max_abs_m, _rw_state["up_m"]*decay + random.uniform(-1,1)*(step*0.2)))
    dlat, dlon, dalt = meters_to_deg_offsets(lat, _rw_state["north_m"], _rw_state["east_m"], _rw_state["up_m"])
    return lat + dlat, lon + dlon, alt + dalt

# 2) circular orbit
def spoof_circular(lat, lon, alt, radius_m=5.0, period_s=20.0, altitude_variation_m=0.5, phase_offset=0.0):
    theta = 2.0 * math.pi * (((time.time() + phase_offset) % period_s) / period_s)
    north_m = math.cos(theta) * radius_m
    east_m  = math.sin(theta) * radius_m
    up_m    = altitude_variation_m * math.cos(theta)
    dlat, dlon, dalt = meters_to_deg_offsets(lat, north_m, east_m, up_m)
    return lat + dlat, lon + dlon, alt + dalt

# 3) sine along heading
def spoof_sine_along_heading(lat, lon, alt, heading_deg=45.0, amplitude_m=8.0, period_s=12.0, vertical_amplitude_m=1.0):
    phi = 2.0 * math.pi * ((time.time() % period_s) / period_s)
    wave = math.sin(phi)
    hdg = math.radians(heading_deg)
    north_m = math.cos(hdg) * amplitude_m * wave
    east_m  = math.sin(hdg) * amplitude_m * wave
    up_m    = vertical_amplitude_m * math.sin(2.0 * phi)
    dlat, dlon, dalt = meters_to_deg_offsets(lat, north_m, east_m, up_m)
    return lat + dlat, lon + dlon, alt + dalt
