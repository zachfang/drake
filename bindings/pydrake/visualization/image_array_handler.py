import argparse
from io import BytesIO 
import tempfile
import threading
import time
import zlib

from flask import Flask, Response
import numpy as np
from PIL import Image

from drake import (
    lcmt_image,
    lcmt_image_array,
)
from pydrake.lcm import (
    DrakeLcm,
)

class _ImageServer(Flask):
    """Streams images via the HTTP protocol given an image source. The image
    source should be a continuously running generator function that yields
    images sequentially.
    """

    def __init__(self, *, image_generator):
        super().__init__("meldis_lcm_image_viewer")
        self.add_url_rule("/", view_func=self._serve_image)

        self._image_generator = image_generator

    def _serve_image(self):
        return Response(
            self._image_generator(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )


class LcmImageArrayHandler:
    """Displays LCM images in another URL that has the same host IP but a
    different port number.

    It listens to `lcmt_image_arry` messages, processes them to image files,
    and store the images to a queue. A flask server, _ImageServer, grabs images
    when available and broadcasts them to the URL for visualization.
    """

    _IMAGE_DATA_TYPE = {
        lcmt_image.CHANNEL_TYPE_INT16: np.int16,
        lcmt_image.CHANNEL_TYPE_UINT8: np.uint8,
        lcmt_image.CHANNEL_TYPE_UINT16: np.uint16,
        lcmt_image.CHANNEL_TYPE_FLOAT32: np.float32,
    }
    """The mapping from `lcmt_image` channel_type enum to numpy data type."""

    _IMAGE_CHANNEL_NUM = {
        lcmt_image.PIXEL_FORMAT_RGBA: 4,
        lcmt_image.PIXEL_FORMAT_DEPTH: 1,
        lcmt_image.PIXEL_FORMAT_LABEL: 1,
    }
    """The mapping from `lcmt_image` pixel_format enum to the number of
    channels.
    """

    def __init__(self, host, port, channel):
        # Pending image files to be sent to the server.
        self._latest_message = None
        self._last_update_time = time.time()

        # Subscribe to the channel.
        self._lcm = DrakeLcm()
        self._lcm.Subscribe(channel=channel, handler=self._update_mesage)

        # Instantiate an `_ImageServer` and run it.
        self._image_server = _ImageServer(image_generator=self.image_generator)
        self._image_server.run(
                host=host, port=port, debug=False, threaded=False
        )

    def image_generator(self):
        while True:
            self._lcm.HandleSubscriptions(timeout_millis=1000)
            if self._latest_message is not None:
                new_image = self._process_message()
                self._latest_message = None
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/png\r\n\r\n"
                    + new_image
                    + b"\r\n"
                )

    def _update_mesage(self, message):
        self._latest_message = message

    def _process_message(self):
        """Update this. Note that only a subset of `lcmt_image` types are
        supported.
        """
        image_array = lcmt_image_array.decode(self._latest_message)
        assert len(image_array.images) > 0

        for image in image_array.images:
            w = image.width
            h = image.height

            # Validate the image.
            data_type = self._IMAGE_DATA_TYPE.get(image.channel_type, None)
            num_channels = self._IMAGE_CHANNEL_NUM.get(
                image.pixel_format, None
            )
            bytes_per_pixel = np.dtype(data_type).itemsize * num_channels
            assert data_type, image.channel_type
            assert num_channels, image.pixel_format
            assert image.row_stride == w * bytes_per_pixel, image.row_stride

            if (
                image.compression_method
                == lcmt_image.COMPRESSION_METHOD_NOT_COMPRESSED
            ):
                data_bytes = image.data
            elif (
                image.compression_method == lcmt_image.COMPRESSION_METHOD_ZLIB
            ):
                # TODO(eric): Consider using `data`s buffer, if possible.
                # Can decompress() somehow use an existing buffer in Python?
                data_bytes = zlib.decompress(image.data)
            else:
                raise RuntimeError(
                    f"Unsupported compression type:{image.compression_method}"
                )

            # Update the image for display in memory.
            buffer = BytesIO()
            pil_image = Image.frombuffer("RGBA", (w, h), data_bytes)
            pil_image.save(buffer, format="png")
            return buffer.getbuffer()


def main():
    parser = argparse.ArgumentParser(description=__doc__,)
    parser.add_argument(
        "--host",
        type=str,
        required=False,
        default="127.0.0.1",
        help="URL to host on, default: 127.0.0.1.",
    )
    parser.add_argument(
        "--port",
        type=int,
        required=False,
        default=8000,
        help="Port to host on, default: 8000.",
    )
    parser.add_argument(
        "--channel",
        type=str,
        required=True,
        help="The LCM channel to subscribe to.",
    )
    args = parser.parse_args()

    image_array_handler = LcmImageArrayHandler(
        args.host, args.port, args.channel
    )

if __name__ == '__main__':
    main()
