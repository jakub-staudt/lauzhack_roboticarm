import hashlib
from dataclasses import dataclass

import cv2
import numpy as np

from arm_dashboard.scan.worker.exceptions import InvalidFrameError


@dataclass(frozen=True)
class DecodedFrame:
    """A captured JPEG, decoded. Internal to the collector; not written as is."""

    image: np.ndarray
    sha256: str
    width: int
    height: int
    sharpness: float


class FrameDecoder:
    """Decodes a JPEG and measures what the frame log records about it."""

    def decode(self, data: bytes) -> DecodedFrame:
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise InvalidFrameError('Invalid JPEG')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return DecodedFrame(
            image=image,
            sha256=hashlib.sha256(data).hexdigest(),
            width=image.shape[1],
            height=image.shape[0],
            sharpness=float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        )
