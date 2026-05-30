# src/simulation/video_sim.py
import mujoco
import numpy as np
import imageio
from tqdm import tqdm
from typing import Union
from pathlib import Path
from .base_sim import BaseSim

class VideoSim(BaseSim):
    """Offscreen rendering via mujoco.Renderer.
    Video FPS and simulation FPS are always independent.
    This constructor accepts either a config dict or a path to a YAML config.
    If a path is provided we rely on BaseSim to load it and populate
    `self.cfg` before using configuration values.
    """
    def __init__(self, config: Union[dict, str, Path]):
        # BaseSim handles loading when a path is passed and sets self.cfg
        super().__init__(config)
        cfg = getattr(self, "cfg", config if isinstance(config, dict) else {})

        self.video_fps = cfg.get("video_fps", 30)
        self.video_path = cfg.get("video_path", "videos/output.mp4")
        self.width = int(cfg.get("video_w", 1280))
        self.height = int(cfg.get("video_h", 720))

        # capture one frame every this many physics steps
        self.capture_every = max(1, int(1.0 / (self.dt * self.video_fps)))
        self._frames: list[np.ndarray] = []

        # Ensure the offscreen framebuffer in the model is large enough.
        # Some URDFs/MJCFs set a small default offwidth/offheight (e.g. 640×480)
        # which causes mujoco.Renderer to raise if we request larger images.
        try:
            vis = getattr(self.model, "vis", None)
            if vis is not None and hasattr(vis, "global_"):
                g = vis.global_
                # If attributes exist and are smaller than requested, update them.
                try:
                    if getattr(g, "offwidth", 0) < self.width:
                        g.offwidth = int(self.width)
                    if getattr(g, "offheight", 0) < self.height:
                        g.offheight = int(self.height)
                except Exception:
                    # Some model objects may not allow attribute assignment; ignore.
                    pass
        except Exception:
            pass

        # Try to create the renderer; if the model's offscreen framebuffer is
        # smaller than requested, fall back to a safe size instead of failing.
        try:
            self._renderer = mujoco.Renderer(self.model, self.height, self.width)
        except ValueError as e:
            # Attempt to read framebuffer limits, choose safe fallback sizes.
            try:
                vis = getattr(self.model, "vis", None)
                g = getattr(vis, "global_", None)
                buffer_width = int(getattr(g, "offwidth", 640))
                buffer_height = int(getattr(g, "offheight", 480))
            except Exception:
                buffer_width, buffer_height = 640, 480

            safe_w = min(self.width, buffer_width)
            safe_h = min(self.height, buffer_height)
            print(
                f"Requested video size {self.width}x{self.height} > framebuffer {buffer_width}x{buffer_height}."
                f" Falling back to {safe_w}x{safe_h}.")
            self.width, self.height = safe_w, safe_h
            self._renderer = mujoco.Renderer(self.model, self.height, self.width)
        
        # create once, e.g. in __init__
        self._camera = mujoco.MjvCamera()

        # optional: free camera
        self._camera.type = mujoco.mjtCamera.mjCAMERA_FREE

    def run(self):
        n_steps = int(self.t_end / self.dt)
        for i in tqdm(range(n_steps), desc="Simulation", unit="step"):
            t = i * self.dt
            self._loop_step(t, i)
            mujoco.mj_step(self.model, self.data)
            self.logger.commit(t)
            if i % self.capture_every == 0:
                self._capture_frame()
        self._write_video()
        self._teardown()

    def _capture_frame(self):
        self._renderer.update_scene(self.data)
        frame = self._renderer.render()          # returns H×W×3 uint8 RGB
        self._frames.append(frame)

    def _write_video(self):
        import pathlib
        pathlib.Path(self.video_path).parent.mkdir(parents=True, exist_ok=True)
        try:
            # Primary attempt: use imageio.mimwrite (requires ffmpeg backend)
            imageio.mimwrite(self.video_path, self._frames, fps=self.video_fps)
            print(f"Video saved → {self.video_path}")
        except Exception as exc:
            # If no suitable backend is available, try to use imageio-ffmpeg
            print("Warning: could not write video with imageio.mimwrite:", exc)
            try:
                import imageio_ffmpeg

                print("Using imageio-ffmpeg as fallback writer.")
                writer = imageio.get_writer(self.video_path, fps=self.video_fps, plugin="ffmpeg")
                for frame in self._frames:
                    writer.append_data(frame)
                writer.close()
                print(f"Video saved → {self.video_path}")
                return
            except Exception as exc2:
                print("Fallback with imageio-ffmpeg failed:", exc2)

            # Final fallback: save frames as individual PNGs so user can assemble them
            seq_dir = pathlib.Path(self.video_path).with_suffix("").with_name(pathlib.Path(self.video_path).stem + "_frames")
            seq_dir.mkdir(parents=True, exist_ok=True)
            for i, frame in enumerate(self._frames):
                fname = seq_dir / f"frame_{i:05d}.png"
                imageio.imwrite(str(fname), frame)
            print(f"Could not write MP4. Saved {len(self._frames)} frames to {seq_dir}.")
            print("To enable MP4 writing install ffmpeg support: pip install 'imageio[ffmpeg]' or pip install imageio-ffmpeg")

    def _teardown(self):
        self._renderer.close()