__all__ = ["main", "launch_lab", "launch_spec", "PRESETS", "LabSettings"]


def main() -> int:
    from swarmpathfinding.viz.live import main as _main

    return _main()


def launch_lab(*args, **kwargs):
    from swarmpathfinding.viz.live import launch_lab as _launch

    return _launch(*args, **kwargs)


def launch_spec(*args, **kwargs):
    from swarmpathfinding.viz.live import launch_spec as _launch

    return _launch(*args, **kwargs)


def __getattr__(name: str):
    if name in {"PRESETS", "LabSettings"}:
        from swarmpathfinding.viz import settings

        return getattr(settings, name)
    raise AttributeError(name)
