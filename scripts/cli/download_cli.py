# scripts/cli/download_cli.py — pteleop download subcommand

from __future__ import annotations

import typer

from .ensure_meshes import (
    ALL_MESH_DIRS,
    ensure_meshes,
    ensure_meshes_all,
    list_variants,
)

download_app = typer.Typer(
    name="download",
    help="Manage robot mesh assets (download from Hugging Face).",
    no_args_is_help=True,
)


def _complete_variant(ctx: typer.Context, args: list[str], incomplete: str) -> list[str]:
    if not incomplete:
        return ALL_MESH_DIRS
    return [d for d in ALL_MESH_DIRS if d.startswith(incomplete)]


@download_app.command("get")
def download_get(
    variant: str | None = typer.Argument(
        None,
        help="Robot variant to download (e.g. adam_u, adam_pro). Omit for --all.",
        autocompletion=_complete_variant,
    ),
    all_variants: bool = typer.Option(False, "--all", help="Download all robot variants."),
    force: bool = typer.Option(False, "--force", "-f", help="Force re-download even if present."),
    revision: str = typer.Option("main", "--revision", "-r", help="HF repo branch/tag/commit."),
) -> None:
    """Download robot mesh files from Hugging Face.

    \b
    Examples:
      pteleop download get adam_u          # download one variant
      pteleop download get --all           # download everything
      pteleop download get adam_u --force  # force re-download
    """
    if all_variants:
        ok = ensure_meshes_all(force=force, revision=revision)
    elif variant:
        ok = ensure_meshes(variant, force=force, revision=revision)
    else:
        typer.echo("Specify a variant name or use --all.")
        raise typer.Exit(1)

    if not ok:
        raise typer.Exit(1)


@download_app.command("list")
def download_list() -> None:
    """Show download status of each robot variant."""
    typer.echo("Robot mesh variants:")
    list_variants()


def register(app: typer.Typer) -> None:
    app.add_typer(download_app, name="download")
