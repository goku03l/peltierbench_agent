"""Shared scoring for simulated and real runs, so the two are directly comparable."""
import math


def run_metrics(t_min, temp, sp, tail_min=40.0):
    """t_min: minutes since the setpoint was applied; temp: water temperature series; sp: setpoint (C).

    Returns reach time to +-0.5 C, overshoot past the setpoint, and the settled behaviour over the last
    `tail_min` minutes (mean offset, spread, fraction within +-0.5 C, on/off cycles are added by the caller).
    """
    n = len(temp)
    if n < 10:
        return {"samples": n}
    reach = next((t for t, v in zip(t_min, temp) if abs(v - sp) <= 0.5), None)
    start = temp[0]
    over = max(((sp - v) if start > sp else (v - sp)) for v in temp)
    over = max(0.0, over)
    t_end = t_min[-1]
    tail = [v for t, v in zip(t_min, temp) if t >= t_end - tail_min]
    if not tail:
        tail = temp[-10:]
    mean = sum(tail) / len(tail)
    sd = math.sqrt(sum((v - mean) ** 2 for v in tail) / len(tail))
    within = sum(1 for v in tail if abs(v - sp) <= 0.5) / len(tail)
    return {
        "samples": n,
        "reach_min": None if reach is None else round(reach, 1),
        "overshoot_c": round(over, 2),
        "tail_offset_c": round(mean - sp, 2),
        "tail_sd_c": round(sd, 3),
        "tail_within_0p5": round(within, 3),
    }


def count_cycles(t_min, drive, tail_min=40.0, on_threshold=1.0):
    """Number of off->on transitions of the output in the last `tail_min` minutes (limit-cycle indicator)."""
    t_end = t_min[-1]
    n = 0
    prev = None
    for t, u in zip(t_min, drive):
        if t < t_end - tail_min:
            prev = abs(u) >= on_threshold
            continue
        on = abs(u) >= on_threshold
        if prev is not None and on and not prev:
            n += 1
        prev = on
    return n


def score(m, time_budget_min):
    """Lower is better. Weights follow the project goal: settle tight, little overshoot, no cycling, reach reasonably fast."""
    if m.get("reach_min") is None:
        reach_pen = 3.0 + 0.02 * time_budget_min
    else:
        reach_pen = 0.02 * m["reach_min"]
    return round(
        1.0 * abs(m["tail_offset_c"])
        + 4.0 * m["tail_sd_c"]
        + 1.5 * max(0.0, m["overshoot_c"] - 0.3)
        + reach_pen
        + 0.05 * m.get("cycles", 0)
        + 1.0 * (1.0 - m["tail_within_0p5"]),
        3,
    )
