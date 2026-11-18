"""Adds Gazebo's LogRecord system to a world SDF so the run's poses are saved for replay.

Run it INSIDE the sim container, on the container's COPY of the world, right before PX4
starts Gazebo (see "Hooking it into your start script" in README.md). The world file in
your repo is never touched, so normal runs record nothing.

    python3 enable_log_record.py <world.sdf> <output dir, e.g. /workspace/data/recordings/20261118-101500/gz>

Two settings here are deliberate (both learned the hard way on Gazebo Harmonic / gz-sim 8):
  * NO <record_resources>: copying meshes into the log segfaults Gazebo
    (LogRecordPrivate::SaveModels -> sdf::Model::LinkCount) as soon as PX4 spawns the drone.
    The replay finds the meshes through GZ_SIM_RESOURCE_PATH instead.
  * <record_period>0.02</record_period>: 50 Hz state instead of every 1 ms physics step.
    Keeps the log small (a few MB per minute) and is still smooth for 25 fps video.
"""
import os
import re
import sys


def main(sdf_path, out_dir):
    if not os.path.exists(sdf_path):
        print(f'enable_log_record: {sdf_path} not found, recording is off for this run.')
        return
    os.makedirs(os.path.dirname(out_dir.rstrip('/')) or '.', exist_ok=True)
    text = open(sdf_path).read()
    # Remove an earlier LogRecord (the container copy may already have one).
    text = re.sub(r'\s*<plugin filename="gz-sim-log-system".*?</plugin>', '', text, flags=re.S)
    plugin = ('\n    <plugin filename="gz-sim-log-system" name="gz::sim::systems::LogRecord">'
              f'<record_path>{out_dir}</record_path>'
              '<record_period>0.02</record_period></plugin>')
    text, n = re.subn(r'(<world name="[^"]*">)', lambda m: m.group(1) + plugin, text, count=1)
    if n != 1:
        sys.exit(f'enable_log_record: no <world> element in {sdf_path}')
    open(sdf_path, 'w').write(text)
    print(f'Recording this run to {out_dir} (replay it with recording/video.sh replay)')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
