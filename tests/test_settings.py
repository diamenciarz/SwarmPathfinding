from swarmpathfinding.generators.packs import pack_specs
from swarmpathfinding.viz.settings import PRESETS, LabSettings, parse_lab_args


def test_parse_preset_and_override():
    settings = parse_lab_args(["--preset", "fog_scout", "--agents", "15", "--no-fow"])
    assert settings.map_kind == "bernoulli"
    assert settings.n_agents == 15
    assert settings.fow is False


def test_parse_pack():
    settings = parse_lab_args(["--pack", "opposite", "--index", "0"])
    specs = pack_specs("opposite", seeds=(0,))
    assert settings.map_kind == specs[0].map_kind
    assert settings.starts == "opposite"


def test_from_spec():
    spec = pack_specs("fow_ghost", seeds=(0,))[0]
    settings = LabSettings.from_spec(spec)
    assert settings.world_kind == "continuous"
    assert settings.map_kind == "movers"
    assert settings.fow is True


def test_presets_exist():
    assert "fog_scout" in PRESETS
    assert "movers_ghost" in PRESETS
    assert PRESETS["utrap"].world_kind == "continuous"
