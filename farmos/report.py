"""Act 4 — render a RunLog into a pictorial farm-map report (self-contained SVG).

Shows the plot boundary, the boustrophedon path, planned vs actually-planted seed
positions, spacing annotations, and a stats header (crop, date, counts, run time,
spacing accuracy). No external dependencies — pure string building.
"""
from __future__ import annotations

import html

from .executor import RunLog

# Agronomic palette (reads cleanly on white)
_INK, _MUTED = "#2f3b24", "#7a8467"
_BORDER, _GRID = "#4b5d3a", "#e8ebe0"
_PATH = "#cfd8bf"
_PLANNED = "#b6c199"
_PLANTED = "#3f7d3a"


def _esc(s) -> str:
    return html.escape(str(s))


def render_svg(log: RunLog, *, target_px: int = 560) -> str:
    cfg = log.config
    plot_w = cfg["plot_w_m"]
    plot_l = cfg["plot_l_m"]

    scale = min(target_px / max(plot_w, 1e-6), (target_px * 1.15) / max(plot_l, 1e-6))
    m_l, m_r, m_t, m_b = 68, 172, 104, 56
    field_w, field_h = plot_w * scale, plot_l * scale
    W = m_l + field_w + m_r
    H = m_t + field_h + m_b

    def sx(x: float) -> float:
        return m_l + x * scale

    def sy(y: float) -> float:                 # flip: field y=0 at the bottom
        return m_t + (plot_l - y) * scale

    parts: list[str] = []
    parts.append(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{H:.0f}" '
        f'viewBox="0 0 {W:.0f} {H:.0f}" font-family="ui-sans-serif,system-ui,Segoe UI,sans-serif">'
    )
    parts.append(f'<rect width="{W:.0f}" height="{H:.0f}" fill="#ffffff"/>')

    # ── Header ──
    st, sm = log.stats, log.summary
    title = f"Seeding Report — {_esc(log.crop or cfg.get('crop',''))}"
    parts.append(f'<text x="{m_l}" y="34" font-size="22" font-weight="700" fill="{_INK}">{title}</text>')
    sub_bits = []
    if log.recommended_date:
        sub_bits.append(f"date {_esc(log.recommended_date)}")
    sub_bits.append(f"plot {plot_w:g}×{plot_l:g} m")
    sub_bits.append(f"{sm['rows']} rows × {sm['seeds_per_row']} = {sm['spots']} spots")
    sub_bits.append(f"{sm['seeds_total']} seeds")
    parts.append(f'<text x="{m_l}" y="56" font-size="13" fill="{_MUTED}">{_esc("  ·  ".join(sub_bits))}</text>')
    stat_line = (f"distance {st['distance_m']:g} m  ·  est. run {st['est_run_time_s']:g} s  ·  "
                 f"planted spacing {st['executed_spacing']['mean_gap_m']*100:.1f} cm avg  ·  "
                 f"max drift {st['max_position_error_m']*100:.1f} cm")
    parts.append(f'<text x="{m_l}" y="76" font-size="12.5" fill="{_INK}">{_esc(stat_line)}</text>')
    parts.append(f'<line x1="{m_l}" y1="88" x2="{W-m_r:.0f}" y2="88" stroke="{_GRID}" stroke-width="1"/>')

    # ── Plot boundary ──
    parts.append(f'<rect x="{sx(0):.1f}" y="{sy(plot_l):.1f}" width="{field_w:.1f}" '
                 f'height="{field_h:.1f}" fill="#fbfcf8" stroke="{_BORDER}" stroke-width="1.5"/>')

    # ── Row gridlines (subtle) ──
    x = cfg["row_gap_m"]
    grid_x = []
    xi = cfg.get("edge_margin_m") or cfg["row_gap_m"] / 2
    while xi < plot_w:
        grid_x.append(xi)
        xi += cfg["row_gap_m"]
    for gx in grid_x:
        parts.append(f'<line x1="{sx(gx):.1f}" y1="{sy(plot_l):.1f}" x2="{sx(gx):.1f}" '
                     f'y2="{sy(0):.1f}" stroke="{_GRID}" stroke-width="1"/>')

    # ── Boustrophedon path (planned order) ──
    if len(log.planned) > 1:
        pts = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in log.planned)
        parts.append(f'<polyline points="{pts}" fill="none" stroke="{_PATH}" '
                     f'stroke-width="2" stroke-linejoin="round"/>')

    # ── Planned (hollow) then planted (solid) ──
    for x, y in log.planned:
        parts.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="3.2" fill="none" '
                     f'stroke="{_PLANNED}" stroke-width="1.4"/>')
    for x, y in log.executed:
        parts.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="3.4" fill="{_PLANTED}"/>')

    # ── Axis labels + scale bar ──
    parts.append(f'<text x="{m_l + field_w/2:.0f}" y="{H-16:.0f}" font-size="12" '
                 f'fill="{_MUTED}" text-anchor="middle">width {plot_w:g} m (across rows)</text>')
    parts.append(f'<text x="20" y="{m_t + field_h/2:.0f}" font-size="12" fill="{_MUTED}" '
                 f'text-anchor="middle" transform="rotate(-90 20 {m_t + field_h/2:.0f})">'
                 f'length {plot_l:g} m (along rows)</text>')

    # ── Legend ──
    lx, ly = W - m_r + 16, m_t + 8
    parts.append(f'<text x="{lx}" y="{ly}" font-size="12.5" font-weight="700" fill="{_INK}">Legend</text>')
    parts.append(f'<circle cx="{lx+7}" cy="{ly+24}" r="3.4" fill="{_PLANTED}"/>'
                 f'<text x="{lx+20}" y="{ly+28}" font-size="12" fill="{_INK}">planted ({len(log.executed)})</text>')
    parts.append(f'<circle cx="{lx+7}" cy="{ly+46}" r="3.2" fill="none" stroke="{_PLANNED}" stroke-width="1.4"/>'
                 f'<text x="{lx+20}" y="{ly+50}" font-size="12" fill="{_INK}">planned ({len(log.planned)})</text>')
    parts.append(f'<line x1="{lx}" y1="{ly+64}" x2="{lx+30}" y2="{ly+64}" stroke="{_PATH}" stroke-width="2"/>'
                 f'<text x="{lx+38}" y="{ly+68}" font-size="12" fill="{_INK}">path</text>')
    if log.rationale:
        parts.append(f'<text x="{lx}" y="{ly+96}" font-size="10.5" fill="{_MUTED}">'
                     f'<tspan x="{lx}" dy="0">Why this plan:</tspan></text>')
        parts += _wrap_tspans(log.rationale, lx, ly + 112, width_chars=26)

    parts.append("</svg>")
    return "\n".join(parts)


def _wrap_tspans(text: str, x: int, y: int, width_chars: int) -> list[str]:
    words, line, lines = text.split(), "", []
    for w in words:
        if len(line) + len(w) + 1 > width_chars:
            lines.append(line); line = w
        else:
            line = f"{line} {w}".strip()
    if line:
        lines.append(line)
    out = [f'<text x="{x}" y="{y}" font-size="10.5" fill="{_MUTED}">']
    for i, ln in enumerate(lines[:6]):
        out.append(f'<tspan x="{x}" dy="{0 if i == 0 else 13}">{_esc(ln)}</tspan>')
    out.append("</text>")
    return out


def save_report(log: RunLog, svg_path: str) -> str:
    svg = render_svg(log)
    with open(svg_path, "w") as f:
        f.write(svg)
    return svg_path
