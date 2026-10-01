"""End-to-end tests for the aviator CameraPacket -> foxglove.RawImage converter.

The assertions here deliberately mirror what Foxglove Studio itself does when it
opens an MCAP file: read ``schema.data`` as a serialized FileDescriptorSet, build
a fresh protobuf pool from it, ``lookupType()`` the channel's schema name, and
decode the payload with the result. If these tests pass, Foxglove renders the
file, because that is the exact code path in its MCAP protobuf reader.
"""

import io
from pathlib import Path
import json
import shutil
import subprocess

import pytest
from google.protobuf import descriptor_pb2, descriptor_pool, message_factory, timestamp_pb2
from mcap.reader import make_reader
from mcap.writer import Writer
from PIL import Image

from mcap_to_foxglove import convert, default_dst, describe

# The upstream contract at
# https://github.com/foxglove/schemas/blob/main/schemas/proto/foxglove/RawImage.proto
# (number, name, wire type, referenced type). width/height/step are fixed32.
UPSTREAM_RAW_IMAGE_FIELDS = [
    (1, "timestamp", "TYPE_MESSAGE", ".google.protobuf.Timestamp"),
    (2, "width", "TYPE_FIXED32", None),
    (3, "height", "TYPE_FIXED32", None),
    (4, "encoding", "TYPE_STRING", None),
    (5, "step", "TYPE_FIXED32", None),
    (6, "data", "TYPE_BYTES", None),
    (7, "frame_id", "TYPE_STRING", None),
]

PIXEL_FORMAT_TO_ENCODING = {"RGB8": "rgb8"}

# Same, for
# https://github.com/foxglove/schemas/blob/main/schemas/proto/foxglove/CompressedVideo.proto
UPSTREAM_COMPRESSED_VIDEO_FIELDS = [
    (1, "timestamp", "TYPE_MESSAGE", ".google.protobuf.Timestamp"),
    (2, "frame_id", "TYPE_STRING", None),
    (3, "data", "TYPE_BYTES", None),
    (4, "format", "TYPE_STRING", None),
]

ANNEXB_START_CODE = b"\x00\x00\x00\x01"


def _camera_packet_descriptor_set() -> bytes:
    """The FileDescriptorSet the C++ logger embeds (schemas/recording/camera_packet.proto)."""
    fdp = descriptor_pb2.FileDescriptorProto()
    fdp.name = "camera_packet.proto"
    fdp.package = "aviator.record.v1"
    fdp.syntax = "proto3"
    msg = fdp.message_type.add()
    msg.name = "CameraPacket"
    f = msg.field.add()
    f.name, f.number, f.type, f.label = "metadata_json", 1, f.TYPE_STRING, f.LABEL_OPTIONAL
    f = msg.field.add()
    f.name, f.number, f.type, f.label = "data", 2, f.TYPE_BYTES, f.LABEL_OPTIONAL
    fds = descriptor_pb2.FileDescriptorSet()
    fds.file.append(fdp)
    return fds.SerializeToString()


def write_source_mcap(
    path, frames, topic="record.camera.cockpit.rgb", encoding="raw", bitstream_format=None
):
    """Write a synthetic mcap shaped like the ones aviator_logger produces.

    ``frames`` is a list of (log_time_ns, width, height, bytes) -- ``bytes`` is
    raw pixels when ``encoding`` is "raw", an h264 payload when it is "h264".
    """
    with open(path, "wb") as fh:
        w = Writer(fh)
        w.start(profile="aviator")
        schema_id = w.register_schema(
            name="aviator.record.v1.CameraPacket",
            encoding="protobuf",
            data=_camera_packet_descriptor_set(),
        )
        channel_id = w.register_channel(topic=topic, message_encoding="protobuf", schema_id=schema_id)
        for sequence, (log_time, width, height, rgb) in enumerate(frames):
            metadata = {
                "width": width,
                "height": height,
                "pixel_format": "RGB8",
                "stride_bytes": width * 3,
                "encoding": encoding,
                "frame_id": sequence,
                "sequence": sequence,
                "sample_mono_us": 1000 + sequence,
            }
            if bitstream_format is not None:
                metadata["bitstream_format"] = bitstream_format
            fds = descriptor_pb2.FileDescriptorSet.FromString(_camera_packet_descriptor_set())
            pool = descriptor_pool.DescriptorPool()
            pool.Add(fds.file[0])
            Msg = message_factory.GetMessageClass(
                pool.FindMessageTypeByName("aviator.record.v1.CameraPacket")
            )
            payload = Msg(metadata_json=json.dumps(metadata), data=rgb).SerializeToString()
            w.add_message(channel_id=channel_id, log_time=log_time, publish_time=log_time, data=payload)
        w.finish()


def read_images(path):
    """Open a converted mcap the way Foxglove does; return [(log_time, channel, message)]."""
    with open(path, "rb") as fh:
        reader = make_reader(fh)
        summary = reader.get_summary()
        assert summary is not None, "output mcap has no summary section"
        schemas = summary.schemas
        assert len(schemas) == 1, f"expected exactly one schema, got {list(schemas.values())}"
        schema = next(iter(schemas.values()))

        pool = descriptor_pool.DescriptorPool()
        fds = descriptor_pb2.FileDescriptorSet.FromString(schema.data)
        for fdp in fds.file:
            pool.Add(fdp)
        desc = pool.FindMessageTypeByName(schema.name)
        Msg = message_factory.GetMessageClass(desc)

        out = []
        for _s, channel, message in reader.iter_messages():
            out.append((message.log_time, channel, Msg.FromString(message.data)))
        return schema, out


@pytest.fixture
def source_mcap(tmp_path):
    w, h = 4, 2
    frames = [
        (1790686813028472000, w, h, bytes(range(0, w * h * 3))),
        (1790686813062699000, w, h, bytes(range(1, w * h * 3 + 1))),
        (1790686813091234000, w, h, bytes([255] * (w * h * 3))),
    ]
    path = tmp_path / "data.images.mcap"
    write_source_mcap(path, frames)
    return path, frames


def test_emits_one_foxglove_raw_image_channel(source_mcap, tmp_path):
    src, frames = source_mcap
    dst = tmp_path / "out.mcap"

    convert(src, dst)

    schema, messages = read_images(dst)
    assert schema.name == "foxglove.RawImage"
    assert schema.encoding == "protobuf"
    assert len(messages) == len(frames)
    topics = {channel.topic for _t, channel, _m in messages}
    assert len(topics) == 1, f"expected a single topic, got {topics}"


def test_pixel_bytes_and_geometry_survive_the_round_trip(source_mcap, tmp_path):
    src, frames = source_mcap
    dst = tmp_path / "out.mcap"

    convert(src, dst)

    _schema, messages = read_images(dst)
    for (src_log_time, width, height, rgb), (log_time, _ch, img) in zip(frames, messages):
        assert (img.width, img.height) == (width, height)
        assert img.encoding == "rgb8"
        assert img.step == width * 3
        assert img.data == rgb, "image payload must be byte-identical to the recorded frame"
        assert log_time == src_log_time


def test_timestamps_carry_source_log_time(source_mcap, tmp_path):
    src, frames = source_mcap
    dst = tmp_path / "out.mcap"

    convert(src, dst)

    _schema, messages = read_images(dst)
    for (src_log_time, *_rest), (_t, _ch, img) in zip(frames, messages):
        expected = timestamp_pb2.Timestamp()
        expected.FromNanoseconds(src_log_time)
        assert img.timestamp.seconds == expected.seconds
        assert img.timestamp.nanos == expected.nanos


def test_jpeg_mode_emits_foxglove_compressed_image(source_mcap, tmp_path):
    src, frames = source_mcap
    dst = tmp_path / "out.mcap"

    convert(src, dst, jpeg=True)

    schema, messages = read_images(dst)
    assert schema.name == "foxglove.CompressedImage"
    assert len(messages) == len(frames)
    for (src_log_time, width, height, _rgb), (_t, _ch, img) in zip(frames, messages):
        assert img.format == "jpeg"
        assert img.frame_id == "0" or img.frame_id.isdigit()
        assert img.timestamp.seconds == src_log_time // 1_000_000_000
        decoded = Image.open(io.BytesIO(img.data))
        assert decoded.mode == "RGB"
        assert decoded.size == (width, height)


def test_jpeg_mode_keeps_a_flat_frame_nearly_intact(source_mcap, tmp_path):
    """A solid-colour frame is the one case JPEG should reproduce closely."""
    src, frames = source_mcap
    dst = tmp_path / "out.mcap"
    _t, _w, _h, flat_rgb = frames[-1]

    convert(src, dst, jpeg=True)

    _schema, messages = read_images(dst)
    _t, _ch, img = messages[-1]
    pixels = Image.open(io.BytesIO(img.data)).tobytes()
    assert max(abs(a - b) for a, b in zip(pixels, flat_rgb)) <= 20


@pytest.fixture
def h264_source_mcap(tmp_path):
    """An mcap like ``data.images.mcap``: h264 payloads, not raw pixels.

    The first payload is a keyframe (SPS+PPS+IDR), the rest are P-frames, so a
    decoder run over the concatenation sees a valid Annex B stream.
    """
    sps_pps_idr = ANNEXB_START_CODE + b"\x67\x42\x00\x1e\x8c" + ANNEXB_START_CODE + b"\x65\x88"
    frames = [
        (1790686813028472000, 1280, 720, sps_pps_idr),
        (1790686813062699000, 1280, 720, ANNEXB_START_CODE + b"\x41\x9a\x01"),
        (1790686813091234000, 1280, 720, ANNEXB_START_CODE + b"\x41\x9a\x02\x03"),
    ]
    path = tmp_path / "video.images.mcap"
    write_source_mcap(path, frames, encoding="h264", bitstream_format="annexb")
    return path, frames


def test_video_mode_emits_foxglove_compressed_video(h264_source_mcap, tmp_path):
    src, frames = h264_source_mcap
    dst = tmp_path / "out.mcap"

    convert(src, dst, video=True)

    schema, messages = read_images(dst)
    assert schema.name == "foxglove.CompressedVideo"
    assert schema.encoding == "protobuf"
    assert len(messages) == len(frames)
    topics = {channel.topic for _t, channel, _m in messages}
    assert topics == {"record.camera.cockpit.rgb"}


def test_video_mode_remuxes_each_annexb_payload_verbatim(h264_source_mcap, tmp_path):
    """The whole point of the video mode: no decode, no re-encode, no pixel maths."""
    src, frames = h264_source_mcap
    dst = tmp_path / "out.mcap"

    convert(src, dst, video=True)

    _schema, messages = read_images(dst)
    for (src_log_time, _w, _h, payload), (log_time, _ch, video) in zip(frames, messages):
        assert video.data == payload
        assert video.format == "h264"
        assert video.timestamp.seconds == src_log_time // 1_000_000_000
        assert video.timestamp.nanos == src_log_time % 1_000_000_000
        assert log_time == src_log_time
        assert video.frame_id == "0" or video.frame_id.isdigit()


def test_compressed_video_descriptor_matches_upstream_contract(h264_source_mcap, tmp_path):
    src, _frames = h264_source_mcap
    dst = tmp_path / "out.mcap"

    convert(src, dst, video=True)

    fdp, fields = read_image_descriptor(dst, "CompressedVideo")
    assert fields == UPSTREAM_COMPRESSED_VIDEO_FIELDS
    assert "google/protobuf/timestamp.proto" in fdp.dependency


def test_raw_mode_refuses_an_h264_source(h264_source_mcap, tmp_path):
    """h264 bytes in a RawImage would render as noise, so refuse loudly."""
    src, _frames = h264_source_mcap

    with pytest.raises(ValueError, match="h264"):
        convert(src, tmp_path / "out.mcap")


def test_video_mode_refuses_a_raw_pixel_source(source_mcap, tmp_path):
    src, _frames = source_mcap

    with pytest.raises(ValueError, match="raw"):
        convert(src, tmp_path / "out.mcap", video=True)


def test_video_mode_refuses_a_length_prefixed_bitstream(tmp_path):
    """Foxglove only accepts Annex B; AVCC payloads would fail to decode in the panel."""
    path = tmp_path / "avcc.mcap"
    write_source_mcap(
        path,
        [(1790686813028472000, 1280, 720, ANNEXB_START_CODE + b"\x65\x88")],
        encoding="h264",
        bitstream_format="avcc",
    )

    with pytest.raises(ValueError, match="annexb"):
        convert(path, tmp_path / "out.mcap", video=True)


def test_video_mode_refuses_a_payload_without_a_start_code(tmp_path):
    path = tmp_path / "no-startcode.mcap"
    write_source_mcap(
        path,
        [(1790686813028472000, 1280, 720, b"\x65\x88\x99")],
        encoding="h264",
        bitstream_format="annexb",
    )

    with pytest.raises(ValueError, match="Annex B"):
        convert(path, tmp_path / "out.mcap", video=True)


def test_video_and_jpeg_modes_are_mutually_exclusive(source_mcap, tmp_path):
    src, _frames = source_mcap

    with pytest.raises(ValueError, match="jpeg"):
        convert(src, tmp_path / "out.mcap", jpeg=True, video=True)


def read_image_descriptor(path, message_name):
    """Pull one message's field list out of the schema a converted file embeds."""
    with open(path, "rb") as fh:
        schema = next(iter(make_reader(fh).get_summary().schemas.values()))
    fds = descriptor_pb2.FileDescriptorSet.FromString(schema.data)
    fdp = next(f for f in fds.file if any(m.name == message_name for m in f.message_type))
    msg = next(m for m in fdp.message_type if m.name == message_name)
    fields = [
        (f.number, f.name, descriptor_pb2.FieldDescriptorProto.Type.Name(f.type), f.type_name or None)
        for f in msg.field
    ]
    return fdp, fields


def test_raw_image_descriptor_matches_upstream_contract(source_mcap, tmp_path):
    src, _frames = source_mcap
    dst = tmp_path / "out.mcap"

    convert(src, dst)

    fdp, fields = read_image_descriptor(dst, "RawImage")
    assert fields == UPSTREAM_RAW_IMAGE_FIELDS
    assert "google/protobuf/timestamp.proto" in fdp.dependency


def test_describe_reports_schema_topics_and_timing(source_mcap):
    src, frames = source_mcap

    info = describe(src)

    assert info["schema"] == "aviator.record.v1.CameraPacket"
    assert info["frames"] == len(frames)
    assert info["topics"] == ["record.camera.cockpit.rgb"]
    assert (info["width"], info["height"], info["pixel_format"]) == (4, 2, "RGB8")
    assert info["duration_s"] == pytest.approx((frames[-1][0] - frames[0][0]) / 1e9)
    assert info["mean_dt_ms"] == pytest.approx(
        (frames[-1][0] - frames[0][0]) / 1e6 / (len(frames) - 1)
    )


def test_describe_reports_the_source_encoding(h264_source_mcap):
    """--info has to tell you whether --video or the raw path is the right one."""
    src, _frames = h264_source_mcap

    info = describe(src)

    assert info["encoding"] == "h264"


REAL_RECORDING = Path("/home/rocos/Downloads/data.images.mcap")


def _real_recording_encoding():
    """Which converter mode the local sample wants, or None if it isn't here.

    The sample gets re-recorded from time to time, and the logger's camera mode
    decides whether its payloads are raw pixels or h264 -- so each real-file test
    below declares which kind it needs instead of assuming one.
    """
    if not REAL_RECORDING.exists():
        return None
    with open(REAL_RECORDING, "rb") as fh:
        reader = make_reader(fh)
        schema = next(iter(reader.get_summary().schemas.values()))
        pool = descriptor_pool.DescriptorPool()
        for fdp in descriptor_pb2.FileDescriptorSet.FromString(schema.data).file:
            pool.Add(fdp)
        Packet = message_factory.GetMessageClass(pool.FindMessageTypeByName(schema.name))
        for _schema, _channel, message in reader.iter_messages():
            metadata = json.loads(Packet.FromString(message.data).metadata_json)
            return metadata.get("encoding", "raw")
    return None


REAL_RECORDING_ENCODING = _real_recording_encoding()

sample_is_raw = pytest.mark.skipif(
    REAL_RECORDING_ENCODING != "raw",
    reason=f"sample holds {REAL_RECORDING_ENCODING!r} payloads, not raw pixels",
)
sample_is_h264 = pytest.mark.skipif(
    REAL_RECORDING_ENCODING != "h264",
    reason=f"sample holds {REAL_RECORDING_ENCODING!r} payloads, not h264",
)



def read_source_frames(path):
    """Read (log_time, pixel_bytes) straight out of a recorded CameraPacket MCAP."""
    with open(path, "rb") as fh:
        reader = make_reader(fh)
        schema = next(iter(reader.get_summary().schemas.values()))
        pool = descriptor_pool.DescriptorPool()
        for fdp in descriptor_pb2.FileDescriptorSet.FromString(schema.data).file:
            pool.Add(fdp)
        Packet = message_factory.GetMessageClass(pool.FindMessageTypeByName(schema.name))
        return [
            (m.log_time, Packet.FromString(m.data).data)
            for _s, _c, m in reader.iter_messages()
        ]


@pytest.mark.skipif(not REAL_RECORDING.exists(), reason="recorded sample not on this machine")
@sample_is_raw
def test_real_recording_round_trips_every_pixel(tmp_path):
    src_frames = read_source_frames(REAL_RECORDING)
    dst = tmp_path / "real.mcap"

    convert(REAL_RECORDING, dst)

    schema, messages = read_images(dst)
    assert schema.name == "foxglove.RawImage"
    assert len(messages) == len(src_frames)
    for (src_log_time, src_rgb), (log_time, _channel, img) in zip(src_frames, messages):
        assert img.data == src_rgb
        assert img.encoding == "rgb8"
        assert img.step == img.width * 3
        assert log_time == src_log_time


@pytest.mark.skipif(not REAL_RECORDING.exists(), reason="recorded sample not on this machine")
@sample_is_h264
def test_real_recording_video_remux_keeps_every_payload(tmp_path):
    src_frames = read_source_frames(REAL_RECORDING)
    dst = tmp_path / "real.mcap"

    convert(REAL_RECORDING, dst, video=True)

    schema, messages = read_images(dst)
    assert schema.name == "foxglove.CompressedVideo"
    assert len(messages) == len(src_frames)
    for (src_log_time, payload), (log_time, _channel, video) in zip(src_frames, messages):
        assert video.data == payload
        assert video.format == "h264"
        assert log_time == src_log_time


FFPROBE = shutil.which("ffprobe")


@pytest.mark.skipif(not REAL_RECORDING.exists(), reason="recorded sample not on this machine")
@pytest.mark.skipif(FFPROBE is None, reason="ffprobe not installed")
@sample_is_h264
def test_remuxed_frames_decode_as_one_frame_per_message(tmp_path):
    """Pin the two properties Foxglove's h264 path needs but our bytes can't show.

    It rejects B frames outright (they need lookahead) and expects exactly one
    frame per message, so decode the remuxed payloads back with ffprobe.
    """
    dst = tmp_path / "real.mcap"
    convert(REAL_RECORDING, dst, video=True)

    _schema, messages = read_images(dst)
    sample = messages[:90]
    stream = tmp_path / "sample.h264"
    stream.write_bytes(b"".join(v.data for _t, _ch, v in sample))

    probe = subprocess.run(
        [
            FFPROBE, "-v", "error", "-select_streams", "v:0", "-count_frames",
            "-show_entries", "stream=codec_name,has_b_frames,nb_read_frames",
            "-of", "json", str(stream),
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    info = json.loads(probe.stdout)["streams"][0]
    assert info["codec_name"] == "h264"
    assert int(info["has_b_frames"]) == 0
    assert int(info["nb_read_frames"]) == len(sample)


@pytest.mark.parametrize(
    "src, expected",
    [
        ("/data/data.images.mcap", "/data/data.images.foxglove.mcap"),
        ("/data/rec.mcap", "/data/rec.foxglove.mcap"),
    ],
)
def test_default_output_name_keeps_the_source_stem(src, expected):
    assert default_dst(Path(src)) == Path(expected)


def test_refuses_encoded_payloads_that_claim_a_raw_pixel_format(tmp_path):
    """The logger's compressed mode still reports pixel_format RGB8.

    Writing those bytes into a RawImage whose encoding says rgb8 would render
    garbage without any error, so the converter must refuse instead.
    """
    path = tmp_path / "h265.mcap"
    write_source_mcap(
        path,
        [(1790686813028472000, 4, 2, b"\x00\x01\x02" * 8)],
        encoding="compressed",
    )

    with pytest.raises(ValueError, match="compressed"):
        convert(path, tmp_path / "out.mcap")
