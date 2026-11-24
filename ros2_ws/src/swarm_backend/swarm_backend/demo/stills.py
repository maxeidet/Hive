"""Stills and GIFs of a built film, for the report and the README (`video.sh final --stills`)."""
import subprocess
from pathlib import Path

MAX_GIF_S, MAX_GIF_W = 8.0, 480


def _frame(clip: Path, at: float):
    from PIL import Image
    import io
    raw = subprocess.run(['ffmpeg', '-v', 'error', '-ss', f'{at:.3f}', '-i', str(clip), '-frames:v', '1',
                          '-f', 'image2pipe', '-vcodec', 'png', '-'], capture_output=True, check=True).stdout
    return Image.open(io.BytesIO(raw)).convert('RGB')


def _duration(clip: Path) -> float:
    out = subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0',
                          str(clip)], capture_output=True, text=True).stdout
    return float(out or 0)


def stills_pdf(scene_clips: list[Path], captions: list[str], out: Path) -> Path:
    """One A4-landscape page per scene: the frame from the middle of its clip, the caption under it."""
    from PIL import Image, ImageDraw, ImageFont
    pages = []
    for clip, cap in zip(scene_clips, captions):
        img = _frame(clip, _duration(clip) / 2)
        page = Image.new('RGB', (1754, 1240), 'white')   # A4 at 150 dpi
        img.thumbnail((1654, 1000))
        page.paste(img, ((1754 - img.width) // 2, 60))
        try:
            font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 36)
        except OSError:
            font = ImageFont.load_default()
        ImageDraw.Draw(page).text((50, 80 + img.height), cap, fill='black', font=font)
        pages.append(page)
    pages[0].save(out, save_all=True, append_images=pages[1:])
    return Path(out)


def gif(clip: Path, out: Path, start: float = 0.0, seconds: float = 6.0, width: int = MAX_GIF_W,
        fps: int = 12) -> tuple[Path, int]:
    """A GIF of part of a clip, capped at 480 px wide and 8 s (palette for small, clean files)."""
    seconds, width = min(seconds, MAX_GIF_S), min(width, MAX_GIF_W)
    vf = f'fps={fps},scale={width}:-2:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse'
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-ss', f'{start:.3f}', '-t', f'{seconds:.3f}', '-i', str(clip),
                    '-filter_complex', vf, str(out)], check=True)
    return Path(out), Path(out).stat().st_size
