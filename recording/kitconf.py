"""Loads recording_config.json (next to this file, or the path in $KIT_CONFIG)."""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def load():
    path = os.environ.get('KIT_CONFIG', os.path.join(HERE, 'recording_config.json'))
    with open(path) as fh:
        cfg = json.load(fh)
    return _strip(cfg)


def _strip(obj):
    """Drops the '_comment' keys so the scripts can iterate over the rest."""
    if isinstance(obj, dict):
        return {k: _strip(v) for k, v in obj.items() if not k.startswith('_')}
    return obj


if __name__ == '__main__':
    # Shell helper:  python3 kitconf.py world  ->  prints cfg['world']
    import sys
    cfg = load()
    for key in sys.argv[1:]:
        cfg = cfg[key]
    print(' '.join(cfg) if isinstance(cfg, list) else cfg)
