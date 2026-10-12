#!/usr/bin/env python3
"""Generate a versioned PCM16 cue pack with local sherpa-onnx Matcha models.

No network access or implicit model downloads. --check requires only PyYAML and
the standard library; inference dependencies are imported only for generation.
"""
import argparse
import array
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import tempfile
import wave

import yaml


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def check_pack(config, directory):
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    if manifest["schema_version"] != 1:
        raise ValueError("unsupported audio manifest")
    for key, cue in config["cues"].items():
        entry = manifest["cues"][key]
        name = entry["file"]
        if not isinstance(name, str) or Path(name).name != name or not name.endswith(".wav"):
            raise ValueError(f"invalid audio path: {key}")
        path = directory / name
        if entry["text"] != cue["text"] or digest(path) != entry["sha256"]:
            raise ValueError(f"stale/corrupt audio: {key}")
        with wave.open(str(path), "rb") as audio:
            if (audio.getsampwidth() != 2 or audio.getnchannels() not in (1, 2)
                    or not 8000 <= audio.getframerate() <= 48000
                    or not 0 < audio.getnframes() <= 30 * audio.getframerate()
                    or len(audio.readframes(audio.getnframes())) != audio.getnframes() * audio.getnchannels() * 2):
                raise ValueError(f"invalid PCM16 WAV: {key}")
    return manifest


def write_wave(path, samples, sample_rate):
    if not 8000 <= sample_rate <= 48000 or not 0 < len(samples) <= 30 * sample_rate:
        raise ValueError("audio must be 8..48 kHz and at most 30 seconds")
    values = array.array("h")
    for sample in samples:
        if not math.isfinite(sample):
            raise ValueError("non-finite TTS sample")
        values.append(round(max(-1.0, min(1.0, float(sample))) * 32767))
    if sys.byteorder != "little":
        values.byteswap()
    with wave.open(str(path), "wb") as audio:
        audio.setparams((1, 2, sample_rate, 0, "NONE", "not compressed"))
        audio.writeframes(values.tobytes())


def build_pack(config, directory, provenance, synthesize):
    """Immutable audio filenames plus an atomically replaced manifest."""
    directory.mkdir(parents=True, exist_ok=True)
    fingerprint = hashlib.sha256(json.dumps(provenance, sort_keys=True, ensure_ascii=False).encode()).hexdigest()
    manifest = {"schema_version": 1, "provenance": provenance, "fingerprint": fingerprint, "cues": {}}
    with tempfile.TemporaryDirectory(prefix=".tts-", dir=directory) as temporary:
        staging = Path(temporary)
        generated = {}
        for key, cue in config["cues"].items():
            text = cue["text"]
            if not isinstance(text, str) or not text or len(text.encode()) > 256 or "\0" in text:
                raise ValueError(f"invalid cue text: {key}")
            if text not in generated:
                target = staging / "pending.wav"
                samples, rate = synthesize(text)
                write_wave(target, samples, rate)
                audio_hash = digest(target)
                # Neural synthesis can be stochastic. Include the resulting bytes
                # so publishing a new pack never overwrites an old manifest's WAV.
                name = hashlib.sha256((fingerprint + "\0" + text + "\0" + audio_hash).encode()).hexdigest() + ".wav"
                target.replace(staging / name)
                generated[text] = {"text": text, "file": name, "sha256": audio_hash}
            manifest["cues"][key] = generated[text]
        (staging / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        check_pack(config, staging)
        for audio in staging.glob("*.wav"):
            audio.replace(directory / audio.name)
        (staging / "manifest.json").replace(directory / "manifest.json")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[2] / "config/tts.yaml")
    parser.add_argument("--output", type=Path, help="override audio_dir from config")
    parser.add_argument("--check", action="store_true", help="check text, WAV format and SHA256 without loading models")
    parser.add_argument("--model-dir", type=Path, help="local Matcha directory (tokens.txt, lexicon.txt, model-steps-3.onnx)")
    parser.add_argument("--vocoder", type=Path, help="local compatible Vocos ONNX file")
    parser.add_argument("--model-source", help="model origin and license note, stored in manifest")
    parser.add_argument("--threads", type=int, choices=(1, 2, 3, 4), default=2)
    parser.add_argument("--speed", type=float, default=1.0)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    directory = (args.output or args.config.resolve().parent / config["audio_dir"]).resolve()
    if args.check:
        check_pack(config, directory)
        print(f"Checked {len(config['cues'])} cues in {directory}")
        return
    if not args.model_dir or not args.vocoder or not args.model_source:
        parser.error("generation requires --model-dir, --vocoder and --model-source")
    if not math.isfinite(args.speed) or not 0.5 <= args.speed <= 2:
        parser.error("--speed must be within 0.5..2")
    model = args.model_dir.resolve()
    for path in (model / "model-steps-3.onnx", model / "tokens.txt", model / "lexicon.txt", args.vocoder):
        if not path.is_file():
            parser.error(f"missing local model file: {path}")
    # Fingerprint includes text normalization/dictionaries as well as weights.
    files = {p.relative_to(model).as_posix(): digest(p) for p in sorted(model.rglob("*")) if p.is_file()}
    import sherpa_onnx
    matcha = sherpa_onnx.OfflineTtsMatchaModelConfig(
        acoustic_model=str(model / "model-steps-3.onnx"), vocoder=str(args.vocoder.resolve()),
        lexicon=str(model / "lexicon.txt"), tokens=str(model / "tokens.txt"),
        dict_dir=str(model / "dict") if (model / "dict").is_dir() else "",
        data_dir=str(model / "espeak-ng-data") if (model / "espeak-ng-data").is_dir() else "")
    rules = ",".join(str(p) for p in sorted(model.glob("*.fst")))
    tts_config = sherpa_onnx.OfflineTtsConfig(
        model=sherpa_onnx.OfflineTtsModelConfig(matcha=matcha, num_threads=args.threads, provider="cpu"),
        rule_fsts=rules, max_num_sentences=1)
    if not tts_config.validate():
        raise ValueError("invalid local sherpa-onnx model configuration")
    engine = sherpa_onnx.OfflineTts(tts_config)
    provenance = {"engine": "sherpa-onnx", "engine_version": importlib.metadata.version("sherpa-onnx"),
                  "model_source": args.model_source, "model_files": files,
                  "vocoder_sha256": digest(args.vocoder), "speed": args.speed, "speaker": 0,
                  "threads": args.threads, "format": "mono PCM16", "generator_version": 1}

    def synthesize(text):
        print(text, flush=True)
        audio = engine.generate(text, sid=0, speed=args.speed)
        return audio.samples, audio.sample_rate

    build_pack(config, directory, provenance, synthesize)
    print(f"Generated {len(config['cues'])} cues in {directory}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, ImportError) as error:
        sys.exit(f"TTS generation: {error}")
