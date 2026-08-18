"""
Central verification entry point for Miniware MDP Control.
"""

from pathlib import Path
import shutil

import nox

nox.options.reuse_existing_virtualenvs = True

_BLACK_TARGETS = ("mdp_control.py", "tests", "noxfile.py")


@nox.session(python=False, default=False)
def clean(session: nox.Session) -> None:
    for pattern in ("build", "dist", ".nox", ".*cache", "*.egg-info", "*.log", "*.tmp"):
        for path in Path.cwd().glob(pattern):
            session.log(f"Removing: {path}")
            if path.is_dir():
                shutil.rmtree(path, ignore_errors=True)
            else:
                path.unlink(missing_ok=True)
    for path in Path.cwd().rglob("__pycache__"):
        session.log(f"Removing: {path}")
        shutil.rmtree(path, ignore_errors=True)


@nox.session
def tests(session: nox.Session) -> None:
    session.install("-e", ".")
    session.run("python", "-m", "unittest", "discover", "-s", "tests", "-v", *session.posargs)


@nox.session
def typecheck(session: nox.Session) -> None:
    session.install("-e", ".", "mypy~=2.1")
    session.run("mypy", *session.posargs)


@nox.session
def black(session: nox.Session) -> None:
    session.install("black~=26.5")
    session.run("python", "-m", "black", *(session.posargs or ("--check", *_BLACK_TARGETS)))


@nox.session
def package(session: nox.Session) -> None:
    session.install("build~=1.5")
    session.run("python", "-m", "build")
