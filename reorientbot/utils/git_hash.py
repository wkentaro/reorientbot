import shlex
import subprocess


def git_hash(cwd=None, log_dir=None):
    cmd = "git diff HEAD --"
    diff = subprocess.check_output(shlex.split(cmd), cwd=cwd).decode()
    if diff:
        if log_dir is None:
            raise RuntimeError(
                "There're changes in git, please commit them first"
            )
        else:
            with open(log_dir / "git.diff", "w") as f:
                f.write(diff)
    cmd = "git log --pretty=format:'%h' -n 1"
    return subprocess.check_output(shlex.split(cmd), cwd=cwd).decode().strip()
