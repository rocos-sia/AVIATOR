#!/usr/bin/env python3
"""Repackage an aviator_logger image MCAP so Foxglove Studio can display it.

The logger records camera frames as ``aviator.record.v1.CameraPacket``
(``metadata_json`` + raw ``data``). Foxglove decodes that schema fine, but its
Image panel only renders a fixed set of image schemas, so a recorded file shows
JSON and no picture. This rewrites the file with the same payload bytes under a
schema the panel does render -- playback, scrubbing and the timeline then come
from Foxglove itself.

Three modes, chosen by the recording's ``encoding``:

``raw`` (default)   pixels -> ``foxglove.RawImage``, lossless
``jpeg``            pixels -> ``foxglove.CompressedImage``, re-encoded
``video``           h264   -> ``foxglove.CompressedVideo``, remuxed untouched
"""

import argparse
import io
import json
import sys
from pathlib import Path

from google.protobuf import (
    descriptor_pb2,
    descriptor_pool,
    message_factory,
    timestamp_pb2,
)
from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer
from PIL import Image

SOURCE_SCHEMA = "aviator.record.v1.CameraPacket"
RAW_IMAGE_SCHEMA = "foxglove.RawImage"
COMPRESSED_IMAGE_SCHEMA = "foxglove.CompressedImage"
COMPRESSED_VIDEO_SCHEMA = "foxglove.CompressedVideo"

# Foxglove's RawImage::encoding uses the ROS image encoding vocabulary.
ENCODING_BY_PIXEL_FORMAT = {"RGB8": "rgb8"}

# Foxglove's Image panel decodes CompressedVideo only when format is h264 and
# the payload is Annex B with one frame per message (see CompressedVideo.proto).
VIDEO_ENCODING = "h264"
ANNEXB_FORMAT = "annexb"
ANNEXB_START_CODES = (b"\x00\x00\x00\x01", b"\x00\x00\x01")

NS_PER_SECOND = 1_000_000_000

_MESSAGE_FIELDS = {
    RAW_IMAGE_SCHEMA: [
        ("timestamp", 1, "TYPE_MESSAGE", ".google.protobuf.Timestamp"),
        ("width", 2, "TYPE_FIXED32", None),
        ("height", 3, "TYPE_FIXED32", None),
        ("encoding", 4, "TYPE_STRING", None),
        ("step", 5, "TYPE_FIXED32", None),
        ("data", 6, "TYPE_BYTES", None),
        ("frame_id", 7, "TYPE_STRING", None),
    ],
    COMPRESSED_IMAGE_SCHEMA: [
        ("timestamp", 1, "TYPE_MESSAGE", ".google.protobuf.Timestamp"),
        ("data", 2, "TYPE_BYTES", None),
        ("format", 3, "TYPE_STRING", None),
        ("frame_id", 4, "TYPE_STRING", None),
    ],
    COMPRESSED_VIDEO_SCHEMA: [
        ("timestamp", 1, "TYPE_MESSAGE", ".google.protobuf.Timestamp"),
        ("frame_id", 2, "TYPE_STRING", None),
        ("data", 3, "TYPE_BYTES", None),
        ("format", 4, "TYPE_STRING", None),
    ],
}


def _foxglove_descriptor_set(schema_name) -> bytes:
    """Build a foxglove schema as a FileDescriptorSet, without protoc.

    Field numbers and wire types are copied from
    https://github.com/foxglove/schemas/tree/main/schemas/proto/foxglove --
    note width/height/step are fixed32, not varint. Getting a wire type wrong
    still "works" against our own embedded descriptor, but produces a file
    that no real foxglove.RawImage reader can decode.
    """
    timestamp_fdp = descriptor_pb2.FileDescriptorProto.FromString(
        timestamp_pb2.DESCRIPTOR.serialized_pb
    )

    fdp = descriptor_pb2.FileDescriptorProto()
    fdp.name = "foxglove_image.proto"
    fdp.package = "foxglove"
    fdp.syntax = "proto3"
    fdp.dependency.append(timestamp_fdp.name)

    msg = fdp.message_type.add()
    msg.name = schema_name.rsplit(".", 1)[-1]
    for name, number, type_name, message_type_name in _MESSAGE_FIELDS[schema_name]:
        field = msg.field.add()
        field.name = name
        field.number = number
        field.type = getattr(descriptor_pb2.FieldDescriptorProto, type_name)
        field.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
        if message_type_name is not None:
            field.type_name = message_type_name

    fds = descriptor_pb2.FileDescriptorSet()
    fds.file.append(timestamp_fdp)
    fds.file.append(fdp)
    return fds.SerializeToString()


def _message_class(schema_data, full_name):
    """Resolve a schema the way Foxglove does: pool + lookupType(full_name)."""
    fds = descriptor_pb2.FileDescriptorSet.FromString(schema_data)
    pool = descriptor_pool.DescriptorPool()
    for fdp in fds.file:
        pool.Add(fdp)
    return message_factory.GetMessageClass(pool.FindMessageTypeByName(full_name))


def _open_source(src):
    """Open a recorded image MCAP and return (reader, Packet class, summary)."""
    fh = open(src, "rb")
    reader = make_reader(fh)
    summary = reader.get_summary()
    if summary is None or not summary.schemas:
        fh.close()
        raise ValueError(f"{src}: no schema section; is this an aviator logger MCAP?")

    source_schema = summary.schemas[next(iter(summary.schemas))]
    if source_schema.name != SOURCE_SCHEMA:
        fh.close()
        raise ValueError(
            f"{src}: unexpected schema {source_schema.name!r}, expected {SOURCE_SCHEMA!r}"
        )
    return fh, reader, _message_class(source_schema.data, source_schema.name)


def describe(src):
    """Summarize a recorded image MCAP without converting it."""
    fh, reader, Packet = _open_source(src)
    try:
        log_times, topics, geometry = [], [], {}
        for _schema, channel, message in reader.iter_messages():
            if not log_times:
                metadata = json.loads(Packet.FromString(message.data).metadata_json)
                geometry = {
                    key: metadata.get(key)
                    for key in ("width", "height", "pixel_format", "stride_bytes")
                }
                geometry["encoding"] = metadata.get("encoding", "raw")
            if channel.topic not in topics:
                topics.append(channel.topic)
            log_times.append(message.log_time)
    finally:
        fh.close()

    span_ns = (log_times[-1] - log_times[0]) if len(log_times) > 1 else 0
    return {
        "schema": SOURCE_SCHEMA,
        "topics": topics,
        "frames": len(log_times),
        "duration_s": span_ns / NS_PER_SECOND,
        "mean_dt_ms": (span_ns / 1e6 / (len(log_times) - 1)) if len(log_times) > 1 else 0.0,
        **geometry,
    }


def _set_timestamp(image, log_time_ns):
    """Write google.protobuf.Timestamp sub-fields.

    Assigning a Timestamp message wholesale is rejected by protobuf's
    well-known-type wrapper, which only accepts a datetime, so set the two
    integer fields directly.
    """
    image.timestamp.seconds = log_time_ns // NS_PER_SECOND
    image.timestamp.nanos = log_time_ns % NS_PER_SECOND


def _to_jpeg(rgb, width, height, step, quality):
    if step != width * 3:
        raise ValueError(
            f"jpeg mode needs unpadded rows, got step {step} for width {width} "
            f"(use the default raw mode for padded images)"
        )
    buffer = io.BytesIO()
    Image.frombytes("RGB", (width, height), rgb).save(
        buffer, format="JPEG", quality=quality
    )
    return buffer.getvalue()


def _annexb_payload(src, metadata, payload):
    """Check an h264 payload is one Foxglove can decode, and hand it back untouched."""
    bitstream_format = metadata.get("bitstream_format", "")
    if bitstream_format != ANNEXB_FORMAT:
        raise ValueError(
            f"{src}: frame {metadata.get('frame_id')} has bitstream_format "
            f"{bitstream_format!r}; foxglove only decodes {ANNEXB_FORMAT!r}, so a "
            f"length-prefixed stream must be rewrapped before remuxing"
        )
    if not payload.startswith(ANNEXB_START_CODES):
        raise ValueError(
            f"{src}: frame {metadata.get('frame_id')} does not begin with an Annex B "
            f"start code; these bytes would fail to decode in the Image panel"
        )
    return payload


def convert(
    src,
    dst,
    *,
    jpeg=False,
    video=False,
    jpeg_quality=90,
    chunk_compression=CompressionType.NONE,
):
    """Rewrite ``src`` (CameraPacket MCAP) into ``dst`` (foxglove image MCAP).

    Default output is ``foxglove.RawImage`` (lossless, one frame per recorded
    frame). With ``jpeg=True`` it is ``foxglove.CompressedImage`` holding JPEG
    frames instead -- much smaller, at the cost of re-encoding. With
    ``video=True`` the recording's own h264 payloads are remuxed into
    ``foxglove.CompressedVideo`` byte for byte: nothing is decoded or re-encoded,
    so the picture is exactly what the camera recorded.

    Returns a small summary dict. ``log_time`` is carried over untouched, so the
    replay keeps the recorder's original cadence.
    """
    if jpeg and video:
        raise ValueError("jpeg and video modes are mutually exclusive; pick one")

    frames = 0
    topic = None

    if video:
        schema_name = COMPRESSED_VIDEO_SCHEMA
    elif jpeg:
        schema_name = COMPRESSED_IMAGE_SCHEMA
    else:
        schema_name = RAW_IMAGE_SCHEMA
    foxglove_fds = _foxglove_descriptor_set(schema_name)
    FoxgloveImage = _message_class(foxglove_fds, schema_name)

    with open(src, "rb") as src_fh, open(dst, "wb") as dst_fh:
        reader = make_reader(src_fh)
        summary = reader.get_summary()
        if summary is None or not summary.schemas:
            raise ValueError(f"{src}: no schema section; is this an aviator logger MCAP?")

        schema_id = next(iter(summary.schemas))
        source_schema = summary.schemas[schema_id]
        if source_schema.name != SOURCE_SCHEMA:
            raise ValueError(
                f"{src}: unexpected schema {source_schema.name!r}, expected {SOURCE_SCHEMA!r}"
            )
        Packet = _message_class(source_schema.data, source_schema.name)

        writer = Writer(dst_fh, compression=chunk_compression)
        writer.start(profile="aviator")
        out_schema_id = writer.register_schema(
            name=schema_name, encoding="protobuf", data=foxglove_fds
        )

        out_channel_id = None
        for _schema, channel, message in reader.iter_messages():
            packet = Packet.FromString(message.data)
            metadata = json.loads(packet.metadata_json)

            if out_channel_id is None:
                topic = channel.topic
                out_channel_id = writer.register_channel(
                    topic=topic, message_encoding="protobuf", schema_id=out_schema_id
                )

            encoding = metadata.get("encoding", "raw")
            frame_id = str(metadata.get("frame_id", ""))

            if video:
                if encoding != VIDEO_ENCODING:
                    raise ValueError(
                        f"{src}: frame {metadata.get('frame_id')} has encoding {encoding!r}; "
                        f"video mode remuxes {VIDEO_ENCODING} frames, so raw frames want the "
                        f"default mode instead"
                    )
                image = FoxgloveImage(
                    data=_annexb_payload(src, metadata, packet.data),
                    format=VIDEO_ENCODING,
                    frame_id=frame_id,
                )
            else:
                if encoding != "raw":
                    raise ValueError(
                        f"{src}: frame {metadata.get('frame_id')} has encoding {encoding!r}; only raw "
                        f"frames can be repacked, a compressed payload (h264/h265/zstd) must be "
                        f"decoded first. Pass video=True to remux h264, record with camera.mode=raw, "
                        f"or add a decoder here."
                    )

                pixel_format = metadata["pixel_format"]
                if pixel_format not in ENCODING_BY_PIXEL_FORMAT:
                    raise ValueError(
                        f"{src}: pixel_format {pixel_format!r} has no image encoding; "
                        f"known formats: {sorted(ENCODING_BY_PIXEL_FORMAT)}"
                    )

                width, height = metadata["width"], metadata["height"]
                step = metadata["stride_bytes"]
                expected = step * height
                if len(packet.data) != expected:
                    raise ValueError(
                        f"{src}: frame {metadata.get('frame_id')} has {len(packet.data)} bytes, "
                        f"expected {expected} ({width}x{height}, step {step})"
                    )

                if jpeg:
                    image = FoxgloveImage(
                        data=_to_jpeg(packet.data, width, height, step, jpeg_quality),
                        format="jpeg",
                        frame_id=frame_id,
                    )
                else:
                    image = FoxgloveImage(
                        width=width,
                        height=height,
                        encoding=ENCODING_BY_PIXEL_FORMAT[pixel_format],
                        step=step,
                        data=packet.data,
                        frame_id=frame_id,
                    )
            _set_timestamp(image, message.log_time)
            writer.add_message(
                channel_id=out_channel_id,
                log_time=message.log_time,
                publish_time=message.publish_time,
                data=image.SerializeToString(),
            )
            frames += 1

        if out_channel_id is None:
            raise ValueError(f"{src}: contains no messages")

        writer.finish()

    return {"frames": frames, "topic": topic, "schema": schema_name}


def default_dst(src):
    """``data.images.mcap`` -> ``data.images.foxglove.mcap``."""
    return src.with_name(src.stem + ".foxglove.mcap")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("src", type=Path, help="input aviator image MCAP")
    parser.add_argument(
        "dst",
        type=Path,
        nargs="?",
        help="output MCAP (default: <src stem>.foxglove.mcap)",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--jpeg",
        action="store_true",
        help=f"write {COMPRESSED_IMAGE_SCHEMA} (jpeg) instead of {RAW_IMAGE_SCHEMA}",
    )
    mode.add_argument(
        "--video",
        action="store_true",
        help=f"remux recorded h264 into {COMPRESSED_VIDEO_SCHEMA}, byte for byte",
    )
    parser.add_argument("--jpeg-quality", type=int, default=90, help="jpeg quality (default 90)")
    parser.add_argument("--info", action="store_true", help="print a summary of src and exit")
    args = parser.parse_args(argv)

    if args.info:
        info = describe(args.src)
        print(f"{args.src}")
        print(f"  schema   {info['schema']}")
        print(f"  topic    {', '.join(info['topics']) or '(none)'}")
        print(
            f"  image    {info.get('width')}x{info.get('height')} "
            f"{info.get('pixel_format')} (stride {info.get('stride_bytes')})"
        )
        print(f"  source   {info.get('encoding', 'raw')} payloads")
        print(f"  frames   {info['frames']}")
        if info["mean_dt_ms"]:
            print(
                f"  timing   {info['duration_s']:.3f} s, mean {info['mean_dt_ms']:.2f} ms/frame "
                f"({1000 / info['mean_dt_ms']:.2f} fps)"
            )
        return 0

    dst = args.dst or default_dst(args.src)
    result = convert(
        args.src,
        dst,
        jpeg=args.jpeg,
        video=args.video,
        jpeg_quality=args.jpeg_quality,
    )
    print(f"{args.src} -> {dst}")
    print(f"  {result['frames']} frames on {result['topic']} as {result['schema']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
