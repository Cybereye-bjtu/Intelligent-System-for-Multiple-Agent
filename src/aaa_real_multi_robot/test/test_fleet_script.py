from pathlib import Path
import subprocess


FLEET_SCRIPT = Path(__file__).resolve().parents[3] / "fleet.sh"


def test_fleet_script_has_valid_posix_shell_syntax():
    subprocess.run(["sh", "-n", str(FLEET_SCRIPT)], check=True)


def test_set_bool_helper_does_not_inject_an_extra_argument():
    content = FLEET_SCRIPT.read_text()
    assert "std_srvs/srv/SetBool +" not in content
    assert (
        'std_srvs/srv/SetBool "{data: $service_value}"'
        in content
    )
