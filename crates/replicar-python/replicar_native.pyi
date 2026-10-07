"""The native part of replicar; use it through the `replicar` package (`replicar.convert`, ...)."""

def convert(
    replay: str,
    output: str,
    *,
    precision: str = "float32",
    groups: list[str] | None = None,
    with_groups: list[str] | None = None,
    all_frames: bool = False,
    meshes_dir: str | None = None,
) -> None: ...
def convert_many(
    replays: list[str],
    output_dir: str,
    *,
    jobs: int | None = None,
    skip_existing: bool = False,
    precision: str = "float32",
    groups: list[str] | None = None,
    with_groups: list[str] | None = None,
    all_frames: bool = False,
    meshes_dir: str | None = None,
) -> list[dict[str, object]]: ...
def resimulate(
    file: str,
    replay: str,
    output: str,
    *,
    precision: str = "float32",
    groups: list[str] | None = None,
    with_groups: list[str] | None = None,
    all_frames: bool = False,
    meshes_dir: str | None = None,
) -> None: ...
def version() -> str: ...
