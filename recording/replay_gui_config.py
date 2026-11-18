"""Writes Gazebo's playback GUI layout with a video recorder that records on SIM TIME.

    python3 replay_gui_config.py <installed playback_gui.config> <out: ~/.gz/sim/<major>/playback_gui.config>

Why this file exists:
  * `gz sim --playback` only enters replay mode with its OWN playback layout. Passing
    `--gui-config my.config` makes it show the start/quick-start menu instead, and closing
    that menu starts an empty world that crashes. So we copy the installed layout to the
    place Gazebo looks for a user override (~/.gz/sim/<major>/playback_gui.config).
  * use_sim_time=true: every video frame is stamped with sim time, so the mp4 plays at the
    run's real speed even if the replay renders slowly.
  * lockstep=false: with lockstep on, the window froze ("not responding") whenever the replay
    was paused or had reached its end, because it waited for a sim tick that never came.
"""
import re
import sys

RECORD = ('<record_video><use_sim_time>true</use_sim_time><lockstep>false</lockstep>'
          '<bitrate>8000000</bitrate></record_video>')


def main(src, dst):
    text = open(src).read()
    block = re.search(r'<plugin filename="VideoRecorder".*?</plugin>', text, flags=re.S)
    if block:
        body = re.sub(r'\s*<record_video>.*?</record_video>', '', block.group(0), flags=re.S)
        body = body.replace('</plugin>', RECORD + '</plugin>')
        text = text[:block.start()] + body + text[block.end():]
    else:
        # Older layouts have no recorder: add one (it shows up as its own small panel).
        text += f'\n<plugin filename="VideoRecorder" name="VideoRecorder">{RECORD}</plugin>\n'
    open(dst, 'w').write(text)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
