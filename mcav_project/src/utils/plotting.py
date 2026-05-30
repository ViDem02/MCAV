"""
src/utils/plotting.py

Shared matplotlib style + figure-saving helpers.

Usage
-----
from src.utils.plotting import set_style, FigureManager

set_style()                          # call once at notebook top

fm = FigureManager(cfg)              # reads figures_dir from config
fm.save(fig, "errors")               # saves to <figures_dir>/errors.{pdf,png}
fm.save_and_show(fig, "3d_path")     # saves + displays inline in notebook
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from pathlib import Path
from typing import Union, Sequence


# ──────────────────────────────────────────────────────────────────────────────
# global style
# ──────────────────────────────────────────────────────────────────────────────

def set_style(usetex: bool = False) -> None:
    """
    Keep Matplotlib close to its default appearance while forcing scientific
    notation for large or small values.
    Set usetex=True when building the LaTeX report (requires a TeX installation).
    """
    plt.rcParams.update({
        "text.usetex": usetex,
        "axes.formatter.use_mathtext": True,
        "axes.formatter.limits": (-3, 3),
        "axes.formatter.useoffset": True,
    })


# ──────────────────────────────────────────────────────────────────────────────
# figure manager
# ──────────────────────────────────────────────────────────────────────────────

class FigureManager:
    """
    Saves figures to a folder specified in the simulation config,
    and optionally displays them inline.

    Parameters
    ----------
    config      : dict loaded from a YAML config file
    base_dir    : root path (usually the project ROOT); figures_dir in
                  config is resolved relative to this
    formats     : file formats to write (default: pdf + png)
    """

    def __init__(
        self,
        config: dict,
        base_dir: Union[str, Path] = ".",
        formats: Sequence[str] = ("pdf", "png"),
    ):
        self.formats = list(formats)

        figures_dir: str = config.get("figures_dir", "figures/default")
        self.out_dir = Path(base_dir) / figures_dir
        self.out_dir.mkdir(parents=True, exist_ok=True)

    # ── core ──────────────────────────────────────────────────────────────────

    def save(
        self,
        fig: plt.Figure,
        name: str,
        *,
        formats: Sequence[str] | None = None,
        close: bool = False,
    ) -> list[Path]:
        """
        Save *fig* as <figures_dir>/<name>.<ext> for each format.

        Returns the list of paths written.
        """
        fmts   = list(formats) if formats is not None else self.formats
        paths  = []
        for ext in fmts:
            p = self.out_dir / f"{name}.{ext}"
            fig.savefig(p)
            paths.append(p)
            print(f"  ↳ saved {p}")
        if close:
            plt.close(fig)
        return paths

    def save_and_show(
        self,
        fig: plt.Figure,
        name: str,
        *,
        formats: Sequence[str] | None = None,
    ) -> list[Path]:
        """
        Save *fig* and display it inline (Jupyter / IPython).
        The figure remains open after this call.
        """
        paths = self.save(fig, name, formats=formats)
        plt.show()          # renders inline in the notebook
        return paths

    # ── convenience factory methods ──────────────────────────────────────────

    @staticmethod
    def error_subplots(
        df,
        error_cols: list[str],
        labels: list[str] | None = None,
        units: list[str] | None = None,
        ncols: int = 2,
    ) -> plt.Figure:
        """
        Build a grid of error-vs-time subplots ready to save.

        Parameters
        ----------
        df         : simulation DataFrame
        error_cols : column names (e.g. ['ex', 'ey', 'ez', 'epsi'])
        labels     : subplot titles (defaults to error_cols)
        units      : y-axis unit strings (e.g. ['m', 'm', 'm', 'rad'])
        ncols      : columns in the subplot grid
        """
        labels = labels or error_cols
        units  = units  or [""] * len(error_cols)
        n      = len(error_cols)
        nrows  = (n + ncols - 1) // ncols

        fig, axs = plt.subplots(nrows, ncols, figsize=(6 * ncols, 4 * nrows))
        axs_flat = axs.flatten() if n > 1 else [axs]

        for i, (col, lbl, unit) in enumerate(zip(error_cols, labels, units)):
            ax = axs_flat[i]
            ax.plot(df["t"], df[col])
            ax.set_title(lbl)
            ax.set_xlabel("t [s]")
            ax.set_ylabel(f"[{unit}]" if unit else "")
            ax.axhline(0, color="black", linewidth=0.5, linestyle="--")

            # enforce scientific tick formatting for these axes
            try:
                ax.ticklabel_format(axis="both", style="sci", scilimits=(0, 0), useOffset=True)
            except Exception:
                pass

        # hide unused axes
        for j in range(n, len(axs_flat)):
            axs_flat[j].set_visible(False)

        fig.tight_layout()
        return fig

    @staticmethod
    def lyapunov_levelsets(
        df,
        x_col: str = "ex",
        y_col: str = "ey",
        *,
        n_levels: int = 15,
        title: str = r"Level sets of $V$ (projected on $e_x$–$e_y$)",
        figsize: tuple = (6, 6),
    ) -> plt.Figure:
        """
        Plot Lyapunov level sets V(ex,ey) = 0.5*(ex²+ey²) with the
        error trajectory overlaid.
        """
        import numpy as np

        fig, ax = plt.subplots(figsize=figsize)

        e_range = max(df[[x_col, y_col]].abs().max()) * 1.2
        ev = np.linspace(-e_range, e_range, 200)
        EX, EY = np.meshgrid(ev, ev)
        VV = 0.5 * (EX**2 + EY**2)

        cp = ax.contour(EX, EY, VV, levels=n_levels, cmap="viridis", alpha=0.5)
        fig.colorbar(cp, ax=ax, label=r"$V(e_x, e_y)$")

        ax.plot(df[x_col], df[y_col], "k-",  linewidth=1.8, label="Error trajectory")
        ax.plot(df[x_col].iloc[0],  df[y_col].iloc[0],  "go", label="Start")
        ax.plot(df[x_col].iloc[-1], df[y_col].iloc[-1], "ro", label="End")

        ax.axhline(0, color="black", linewidth=0.5)
        ax.axvline(0, color="black", linewidth=0.5)
        ax.set_xlabel(r"$e_x$ [m]")
        ax.set_ylabel(r"$e_y$ [m]")
        ax.set_title(title)
        ax.set_aspect("equal")
        ax.legend()
        # enforce scientific tick formatting on levelset axes
        try:
            ax.ticklabel_format(axis="both", style="sci", scilimits=(0, 0), useOffset=True)
        except Exception:
            pass

        fig.tight_layout()
        return fig

    @staticmethod
    def path_3d(
        df,
        *,
        x_col: str  = "x",  y_col: str  = "y",  z_col: str  = "z",
        xd_col: str = "xd", yd_col: str = "yd", zd_col: str = "zd",
        title: str = "3D Path Following",
        figsize: tuple = (8, 8),
    ) -> plt.Figure:
        """3D actual-vs-reference path plot."""
        fig = plt.figure(figsize=figsize)
        ax  = fig.add_subplot(111, projection="3d")

        ax.plot(df[xd_col], df[yd_col], df[zd_col], "r--", label="Reference")
        ax.plot(df[x_col],  df[y_col],  df[z_col],  "b-",  label="Actual")

        ax.scatter([df[x_col].iloc[0]],  [df[y_col].iloc[0]],  [df[z_col].iloc[0]],
                   color="black", marker="o", label="Start (actual)")
        ax.scatter([df[xd_col].iloc[0]], [df[yd_col].iloc[0]], [df[zd_col].iloc[0]],
                   color="green", marker="x", label="Start (ref)")

        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Z [m]")
        ax.set_title(title)
        ax.legend()
        # enforce scientific tick formatting on 3D axes
        try:
            ax.ticklabel_format(axis="x", style="sci", scilimits=(0, 0), useOffset=True)
            ax.ticklabel_format(axis="y", style="sci", scilimits=(0, 0), useOffset=True)
            if hasattr(ax, "zaxis"):
                ax.zaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
        except Exception:
            pass

        fig.tight_layout()
        return fig