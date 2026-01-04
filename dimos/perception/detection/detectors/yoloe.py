# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from enum import Enum
from typing import TYPE_CHECKING

from ultralytics import YOLOE

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.detectors.types import Detector
from dimos.perception.detection.type import ImageDetections2D
from dimos.utils.data import get_data
from dimos.utils.gpu_utils import is_cuda_available
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

logger = setup_logger()


class YoloePromptMode(Enum):
    """YOLO-E prompt modes for open-vocabulary detection."""

    TEXT = "text"  # Text prompting using RepRTA
    VISUAL = "visual"  # Visual prompting using SAVPE
    LRPC = "lrpc"  # Prompt-free mode using LRPC (1200+ categories)


class Yoloe2DDetector(Detector):
    def __init__(
        self,
        model_path: str = "models_yoloe",
        model_name: str | None = None,
        device: str | None = None,
        prompt_mode: YoloePromptMode = YoloePromptMode.LRPC,
        text_prompts: list[str] | None = None,
        visual_prompts_bboxes: "NDArray[np.float64] | None" = None,
        visual_prompts_cls: "NDArray[np.int64] | None" = None,
    ) -> None:
        """
        Initialize YOLO-E 2D detector with configurable prompt modes.

        Args:
            model_path: Path to model directory (fetched via get_data from LFS).
            model_name: Model filename. For LRPC mode, use *-pf.pt models.
                        Defaults based on prompt_mode.
            device: Device to run inference on ('cuda', 'cpu', or None for auto)
            prompt_mode: Detection mode (TEXT, VISUAL, or LRPC)
            text_prompts: List of class names for TEXT mode (e.g., ["person", "car"])
            visual_prompts_bboxes: Bounding boxes for VISUAL mode, shape (N, 4) in xyxy format
            visual_prompts_cls: Class IDs for VISUAL mode, shape (N,)
        """
        # Select default model based on prompt mode
        if model_name is None:
            if prompt_mode == YoloePromptMode.LRPC:
                # LRPC requires prompt-free model with built-in embeddings
                model_name = "yoloe-11s-seg-pf.pt"
            else:
                model_name = "yoloe-11s-seg.pt"

        # Load model from LFS-managed data directory
        self.model = YOLOE(get_data(model_path) / model_name)

        self.prompt_mode = prompt_mode
        self.text_prompts = text_prompts
        self.visual_prompts_bboxes = visual_prompts_bboxes
        self.visual_prompts_cls = visual_prompts_cls

        if device:
            self.device = device
        elif is_cuda_available():
            self.device = "cuda"
            logger.debug("Using CUDA for YOLO-E 2d detector")
        else:
            self.device = "cpu"
            logger.debug("Using CPU for YOLO-E 2d detector")

        # Configure model based on prompt mode
        self._configure_prompt_mode()

    def _configure_prompt_mode(self) -> None:
        """Configure the model based on the selected prompt mode."""
        if self.prompt_mode == YoloePromptMode.TEXT:
            if not self.text_prompts:
                raise ValueError("text_prompts must be provided for TEXT mode")
            self.model.set_classes(self.text_prompts, self.model.get_text_pe(self.text_prompts))
            logger.debug(f"Configured TEXT mode with classes: {self.text_prompts}")

        elif self.prompt_mode == YoloePromptMode.VISUAL:
            if self.visual_prompts_bboxes is None or self.visual_prompts_cls is None:
                raise ValueError(
                    "visual_prompts_bboxes and visual_prompts_cls must be provided for VISUAL mode"
                )
            self._visual_prompts = {
                "bboxes": self.visual_prompts_bboxes,
                "cls": self.visual_prompts_cls,
            }
            logger.debug("Configured VISUAL mode with provided prompts")

        elif self.prompt_mode == YoloePromptMode.LRPC:
            # LRPC mode uses internal embeddings, no configuration needed
            logger.debug("Configured LRPC (prompt-free) mode")

    def set_text_prompts(self, text_prompts: list[str]) -> None:
        """
        Update text prompts for TEXT mode.

        Args:
            text_prompts: List of class names to detect
        """
        self.text_prompts = text_prompts
        self.prompt_mode = YoloePromptMode.TEXT
        self.model.set_classes(text_prompts, self.model.get_text_pe(text_prompts))
        logger.debug(f"Updated TEXT mode with classes: {text_prompts}")

    def set_visual_prompts(
        self,
        bboxes: "NDArray[np.float64]",
        cls: "NDArray[np.int64]",
    ) -> None:
        """
        Update visual prompts for VISUAL mode.

        Args:
            bboxes: Bounding boxes in xyxy format, shape (N, 4)
            cls: Class IDs, shape (N,)
        """
        self.visual_prompts_bboxes = bboxes
        self.visual_prompts_cls = cls
        self._visual_prompts = {"bboxes": bboxes, "cls": cls}
        self.prompt_mode = YoloePromptMode.VISUAL
        logger.debug("Updated VISUAL mode with new prompts")

    def process_image(self, image: Image) -> ImageDetections2D:
        """
        Process an image and return detection results.

        Args:
            image: Input image

        Returns:
            ImageDetections2D containing all detected objects
        """
        track_kwargs = {
            "source": image.to_opencv(),
            "device": self.device,
            "conf": 0.75,
            "iou": 0.6,
            "persist": True,
            "verbose": False,
        }

        if self.prompt_mode == YoloePromptMode.VISUAL:
            track_kwargs["visual_prompts"] = self._visual_prompts

        results = self.model.track(**track_kwargs)

        return ImageDetections2D.from_ultralytics_result(image, results)

    def stop(self) -> None:
        """
        Clean up resources used by the detector, including tracker threads.
        """
        if hasattr(self.model, "predictor") and self.model.predictor is not None:
            predictor = self.model.predictor
            if hasattr(predictor, "trackers") and predictor.trackers:
                for tracker in predictor.trackers:
                    if hasattr(tracker, "tracker") and hasattr(tracker.tracker, "gmc"):
                        gmc = tracker.tracker.gmc
                        if hasattr(gmc, "executor") and gmc.executor is not None:
                            gmc.executor.shutdown(wait=True)
            self.model.predictor = None
