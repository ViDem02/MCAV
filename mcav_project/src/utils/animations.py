"""
src/utils/2d_animations.py

Reusable 2D matplotlib animations for top-down vehicle visualisation.
Each function returns a matplotlib.animation.FuncAnimation object —
the caller decides whether to display it (HTML5) or save it.
"""

from __future__ import annotations

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import animation
from typing import Optional
import base64
from pathlib import Path
from tempfile import TemporaryDirectory
import matplotlib as mpl
from matplotlib.animation import writers
from tqdm import tqdm


# ──────────────────────────────────────────────────────────────────────────────
# helpers
# ──────────────────────────────────────────────────────────────────────────────

def _heading_line(x: float, y: float, angle: float, length: float = 0.5):
    """Return ([x0, x1], [y0, y1]) for a heading indicator arrow."""
    return (
        [x, x + length * np.cos(angle)],
        [y, y + length * np.sin(angle)],
    )


# ──────────────────────────────────────────────────────────────────────────────
# public API
# ──────────────────────────────────────────────────────────────────────────────

def quadcopter_topdown(
    df: pd.DataFrame,
    dt: float,
    *,
    x_col: str       = "x",
    y_col: str       = "y",
    yaw_col: str     = "yaw",
    xd_col: str      = "xd",
    yd_col: str      = "yd",
    psid_col: str    = "psid",
    decimate: int    = 50,
    heading_len: float = 0.5,
    xlim: tuple[float, float] = (-5.0, 5.0),
    ylim: tuple[float, float] = (-5.0, 5.0),
    title: str       = "Quadcopter Top-Down View",
    figsize: tuple[float, float] = (8.0, 8.0),
    fig: Optional[plt.Figure] = None,
    ax:  Optional[plt.Axes]  = None,
) -> animation.FuncAnimation:
    """
    Animate a top-down (XY) view of actual vs reference trajectory.

    Parameters
    ----------
    df        : simulation DataFrame (from SimLogger.to_dataframe())
    dt        : physics timestep [s]  — used to set animation interval
    decimate  : keep every Nth row to control animation speed/length
    heading_len : length of the heading arrow in metres

    Returns
    -------
    matplotlib.animation.FuncAnimation  (call .to_html5_video() or .save())
    """
    df_anim = df.iloc[::decimate].reset_index(drop=True)

    if fig is None or ax is None:
        fig, ax = plt.subplots(figsize=figsize)

    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    # ── artists ──
    actual_traj, = ax.plot([], [], "b-",  alpha=0.5, label="Actual")
    ref_traj,    = ax.plot([], [], "r--", alpha=0.5, label="Reference")
    actual_dot,  = ax.plot([], [], "bo",  markersize=7)
    ref_dot,     = ax.plot([], [], "rx",  markersize=7)
    actual_head, = ax.plot([], [], "b-",  linewidth=2)
    ref_head,    = ax.plot([], [], "r-",  linewidth=2)
    time_text    = ax.text(0.02, 0.95, "", transform=ax.transAxes, fontsize=10)

    ax.legend(loc="upper right")

    _artists = (actual_traj, ref_traj, actual_dot, ref_dot,
                actual_head, ref_head, time_text)

    def _init():
        for a in _artists:
            if hasattr(a, "set_data"):
                a.set_data([], [])
        time_text.set_text("")
        return _artists

    def _update(frame: int):
        hist = df_anim.iloc[: frame + 1]
        cur  = df_anim.iloc[frame]

        actual_traj.set_data(hist[x_col],  hist[y_col])
        ref_traj.set_data(hist[xd_col], hist[yd_col])

        actual_dot.set_data([cur[x_col]],  [cur[y_col]])
        ref_dot.set_data(   [cur[xd_col]], [cur[yd_col]])

        hx, hy = _heading_line(cur[x_col],  cur[y_col],  cur[yaw_col],  heading_len)
        rx, ry = _heading_line(cur[xd_col], cur[yd_col], cur[psid_col], heading_len)
        actual_head.set_data(hx, hy)
        ref_head.set_data(rx, ry)

        time_text.set_text(f"t = {cur['t']:.2f} s")
        return _artists

    interval_ms = decimate * dt * 1000.0   # wall-clock ms per frame

    anim = animation.FuncAnimation(
        fig,
        _update,
        frames=len(df_anim),
        init_func=_init,
        blit=True,
        interval=interval_ms,
    )

    plt.close(fig)   # prevent double-display in notebooks
    return anim


def error_convergence(
    df: pd.DataFrame,
    error_cols: list[str],
    labels: Optional[list[str]] = None,
    *,
    title: str = "Error Convergence",
    figsize: tuple[float, float] = (10.0, 4.0),
    fig: Optional[plt.Figure] = None,
    ax:  Optional[plt.Axes]  = None,
) -> animation.FuncAnimation:
    """
    Animate error signals growing in time (useful for convergence demos).

    Parameters
    ----------
    error_cols : list of column names in df  (e.g. ['ex', 'ey', 'ez'])
    labels     : display names (defaults to error_cols)
    """
    labels = labels or error_cols

    if fig is None or ax is None:
        fig, ax = plt.subplots(figsize=figsize)

    ax.set_xlim(df["t"].iloc[0], df["t"].iloc[-1])
    y_max = df[error_cols].abs().max().max() * 1.2
    ax.set_ylim(-y_max, y_max)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Error")
    ax.set_title(title)
    ax.axhline(0, color="black", linewidth=0.5, linestyle="--")
    ax.grid(True)

    lines = [ax.plot([], [], label=lbl)[0] for lbl in labels]
    ax.legend()

    step = max(1, len(df) // 300)   # ~300 animation frames regardless of df length
    df_anim = df.iloc[::step].reset_index(drop=True)

    def _init():
        for ln in lines:
            ln.set_data([], [])
        return lines

    def _update(frame: int):
        hist = df_anim.iloc[: frame + 1]
        for ln, col in zip(lines, error_cols):
            ln.set_data(hist["t"], hist[col])
        return lines

    anim = animation.FuncAnimation(
        fig, _update, frames=len(df_anim),
        init_func=_init, blit=True, interval=30,
    )
    plt.close(fig)
    return anim


def to_html5_video_with_progress(anim: animation.FuncAnimation, embed_limit: Optional[float] = None, *, desc: str = "Rendering animation") -> str:
    """
    Save an animation to an H264 (mp4) in-memory HTML5 video tag while
    displaying a tqdm progress bar during frame generation.

    Parameters
    ----------
    anim : matplotlib.animation.FuncAnimation
        The animation to save.
    embed_limit : float, optional
        Maximum embed size in MB (same semantics as matplotlib's
        Animation.to_html5_video). If the generated video exceeds the
        limit, the function returns the string 'Video too large to embed.'
    desc : str
        Description shown in the tqdm bar.

    Returns
    -------
    str
        HTML5 `<video>` tag with base64-embedded mp4 video, or the
        'Video too large to embed.' string when the embed limit is
        exceeded.
    """
    VIDEO_TAG = r'''<video {size} {options}>
  <source type="video/mp4" src="data:video/mp4;base64,{video}">
  Your browser does not support the video tag.
</video>'''

    # determine embed limit (MB -> bytes)
    embed_limit = mpl._val_or_rc(embed_limit, 'animation.embed_limit')
    embed_limit_bytes = embed_limit * 1024 * 1024

    with TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "temp.m4v"

        # instantiate writer similar to matplotlib's implementation
        Writer = writers[mpl.rcParams['animation.writer']]
        writer = Writer(codec='h264',
                        bitrate=mpl.rcParams['animation.bitrate'],
                        fps=(1000.0 / getattr(anim, '_interval', 200)))

        # progress bar state
        bar = tqdm(total=None, desc=desc, unit='frame')
        last = {'n': 0}

        def _progress_callback(current_frame, total_frames):
            # total_frames may be None
            if total_frames is not None and bar.total is None:
                bar.total = total_frames
            # current_frame is zero-based index; we want count
            cur_count = (current_frame + 1) if current_frame is not None else None
            if cur_count is None:
                return
            delta = cur_count - last['n']
            if delta > 0:
                bar.update(delta)
                last['n'] = cur_count

        try:
            anim.save(str(path), writer=writer, progress_callback=_progress_callback)
        finally:
            bar.close()

        vid64 = base64.encodebytes(path.read_bytes())

    vid_len = len(vid64)
    if vid_len >= embed_limit_bytes:
        mpl._log.warning(
            "Animation movie is %s bytes, exceeding the limit of %s.", vid_len, embed_limit_bytes
        )
        return 'Video too large to embed.'

    video_str = vid64.decode('ascii')
    video_size = 'width="{}" height="{}"'.format(*writer.frame_size)

    options = ['controls', 'autoplay']
    if getattr(anim, '_repeat', False):
        options.append('loop')

    return VIDEO_TAG.format(video=video_str, size=video_size, options=' '.join(options))
