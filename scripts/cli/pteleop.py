#!/usr/bin/env python3
"""PND Teleop CLI.

Usage:
  pteleop                     # start teleop with defaults
  pteleop teleop [OPTIONS] [adam_type] [mocap_driver] [algorithm]
  pteleop launch <launch_stem> # launch a bringup launch file (Tab-completable)
  pteleop setup ssh            # setup SSH key and login
"""

from typing import Annotated

import typer

from .common import get_version
from .download_cli import register as _register_download
from .launch_cli import register as _register_launch
from .self_update_cli import register as _register_self_update
from .setup_cli import register as _register_setup
from .teleop_cli import register as _register_teleop
from .teleop_cli import teleop

app = typer.Typer(
    name="pteleop",
    help="PND Teleop launcher: start nodes by ADAM type, mocap driver and algorithm.",
    add_completion=True,
    no_args_is_help=False,
)

_register_setup(app)
_register_teleop(app)
_register_launch(app)
_register_download(app)
_register_self_update(app)


def _version_callback(value: bool) -> None:
    if value:
        typer.echo(f"pteleop {get_version()}")
        raise typer.Exit()


@app.callback(invoke_without_command=True)
def main(
    ctx: typer.Context,
    version: Annotated[
        bool | None,
        typer.Option("--version", "-V", callback=_version_callback, is_eager=True,
                     help="Show version and exit."),
    ] = None,
) -> None:
    """If no subcommand is given, start teleop with defaults."""
    if ctx.invoked_subcommand is None:
        ctx.invoke(
            teleop,
            adam_type=None,
            mocap_driver=None,
            algorithm=None,
            _adam_opt=None,
            _mocap_opt=None,
            _algo_opt=None,
        )


if __name__ == "__main__":
    app()
