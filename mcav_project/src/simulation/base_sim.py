# src/simulation/base_sim.py
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Union
import mujoco
import tempfile
import os
from tqdm import tqdm
from src.logging.sim_logger import SimLogger
from src.utils.config import load_config

class BaseSim(ABC):
    def __init__(self, config: Union[dict, str, Path]):
        config, config_path = self._load_config_if_needed(config)

        self.cfg = config
        self.dt = config["dt"]
        self.t_end = config["t_end"]
        self.logger = SimLogger()

        self._normalize_paths(config, config_path)
        self._init_model(config["model_path"])    

    def _load_config_if_needed(self, config: Union[dict, str, Path]):
        """Load YAML when a path is provided, return (config_dict, config_path or None)."""
        config_path = None
        if isinstance(config, (str, Path)):
            config_path = Path(config).expanduser().resolve()
            config = load_config(str(config_path))
        return config, config_path

    def _normalize_paths(self, config: dict, config_path: Path | None):
        """Make model/video paths absolute.

        If a config file was provided, interpret relative paths relative to the
        project root (assumed to be the parent of the config folder). This
        prevents resolving relative model paths under the `config/` directory.
        """
        if config_path:
            # config_path is e.g. /.../mcav_project/config/unicycle_fall_north.yaml
            # project_root should be /.../mcav_project
            project_root = config_path.parent.parent
        else:
            project_root = Path.cwd()

        model_path = Path(config["model_path"])
        if not model_path.is_absolute():
            model_path = project_root / model_path
        config["model_path"] = str(model_path)

        if "video_path" in config:
            video_path = Path(config["video_path"])
            if not video_path.is_absolute():
                config["video_path"] = str(project_root / video_path)

    def _init_model(self, model_path: str):
        """Initialize MuJoCo model and data from an XML/URDF path."""
        model_path = str(model_path)

        # If the model file is URDF and contains characters before the XML
        # declaration (e.g. an editor comment), some XML parsers fail. Read
        # the file and sanitize it into a temporary file that starts with an
        # XML declaration. Keep the temp path to clean up at teardown.
        suffix = os.path.splitext(model_path)[1].lower()
        tmp_path = None
        if suffix in (".urdf", ".xml"):
            with open(model_path, "r", encoding="utf-8") as f:
                txt = f.read()

            # Find first XML declaration
            idx = txt.find("<?xml")
            if idx == -1:
                # No declaration — if it starts with '<robot', add declaration
                if txt.lstrip().startswith("<robot"):
                    clean = "<?xml version=\"1.0\"?>\n" + txt
                else:
                    clean = txt
            else:
                # Strip any leading text before the XML declaration
                clean = txt[idx:]

            # Write sanitized content to a temporary file
            tf = tempfile.NamedTemporaryFile(delete=False, suffix=".xml", mode="w", encoding="utf-8")
            tf.write(clean)
            tf.flush()
            tf.close()
            tmp_path = tf.name
            model_path_to_load = tmp_path
        else:
            model_path_to_load = model_path

        try:
            self.model = mujoco.MjModel.from_xml_path(model_path_to_load)
        finally:
            # store tmp path so we can remove it in teardown
            self._tmp_model_path = tmp_path

        self.model.opt.timestep = self.dt
        self.data = mujoco.MjData(self.model)

    # ── override this ──────────────────────────────────────────
    def _loop_step(self, t: float, step: int):
        """
        Apply controls, read state, call self.logger.log(dict).
        Do NOT call mj_step here — the wrapper does it.
        """
        pass
    # ───────────────────────────────────────────────────────────

    def run(self):
        n_steps = int(self.t_end / self.dt)
        for i in tqdm(range(n_steps), desc="Simulation", unit="step"):
            t = i * self.dt
            self._loop_step(t, i)
            mujoco.mj_step(self.model, self.data)
            self.logger.commit(t)
        self._teardown()

    def _teardown(self):
        # Remove any temporary model file we created
        tmp = getattr(self, "_tmp_model_path", None)
        if tmp and os.path.exists(tmp):
            try:
                os.remove(tmp)
            except Exception:
                pass

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)

    def to_dataframe(self):
        return self.logger.to_dataframe()

    def save(self, path: str):
        self.logger.save(path)