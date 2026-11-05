"""python3 -m swarm_backend.scenarios NAME [--sim] [args]: run one scenario."""
import importlib
import sys

from swarm_backend.scenarios import SCENARIOS


def usage() -> str:
    lines = ['Usage: python3 -m swarm_backend.scenarios NAME [--sim] [args]  (NAME --help)', '']
    for name, (module, sim) in SCENARIOS.items():
        doc = importlib.import_module(module).__doc__.splitlines()[0]
        lines.append(f'  {name:20} {"gazebo, sim" if sim else "gazebo":12} {doc}')
    return '\n'.join(lines)


def main(argv: list[str]) -> int:
    if not argv or argv[0] in ('-h', '--help'):
        print(usage())
        return 0
    name, rest = argv[0].replace('-', '_'), argv[1:]
    if name not in SCENARIOS:
        print(f'Unknown scenario: {argv[0]}\n\n{usage()}')
        return 1
    module, sim = SCENARIOS[name]
    if '--sim' in rest and not sim:
        print(f'{name} has no fast simulation yet: it needs Gazebo')
        return 1
    return importlib.import_module(module).main(rest)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
