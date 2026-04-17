import socket
import subprocess
import urllib.error
import urllib.request

try:
    from .common import JETSON_HOST, JETSON_USER, get_ssh_key, jetson_ssh_python_exec
except ImportError:
    from common import JETSON_HOST, JETSON_USER, get_ssh_key, jetson_ssh_python_exec

URL_TEST = "https://www.baidu.com/"
JETSON_SSH_PORT = 22


def _check_tcp_reachable(host, port, timeout):
    """仅检测 TCP 端口是否可达。"""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except (TimeoutError, OSError):
        return False


def _check_ssh_auth(host, user, ssh_key, timeout):
    """非交互式验证 SSH 是否可以成功登录 Jetson。"""
    if not ssh_key:
        return False
    ssh_cmd = [
        "ssh",
        "-i",
        ssh_key,
        "-o",
        "StrictHostKeyChecking=no",
        "-o",
        "BatchMode=yes",
        "-o",
        f"ConnectTimeout={int(timeout)}",
        "-o",
        "LogLevel=ERROR",
        f"{user}@{host}",
        "true",
    ]
    try:
        result = subprocess.run(
            ssh_cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return result.returncode == 0
    except (subprocess.TimeoutExpired, OSError):
        return False


def check_nuc_to_jetson_connection(
    host=JETSON_HOST,
    port=JETSON_SSH_PORT,
    timeout=2,
    user=JETSON_USER,
    ssh_key=None,
    check_ssh_auth=True,
):
    """检查 NUC 是否可以连接到 Jetson。

    先检测 SSH 端口 TCP 可达性; 若 ``check_ssh_auth`` 为 True,
    再用 BatchMode 的 SSH 非交互式登录验证密钥鉴权是否通过。
    只有两步都成功才返回 True, 避免后续对 Jetson 的 SSH/SCP 调用因
    鉴权失败而阻塞或报错 (例如交互式弹出密码提示)。
    """
    if not _check_tcp_reachable(host, port, timeout):
        return False

    if not check_ssh_auth:
        return True

    if ssh_key is None:
        ssh_key = get_ssh_key()
    return _check_ssh_auth(host=host, user=user, ssh_key=ssh_key, timeout=timeout)


def check_local_network_connection(test_url=URL_TEST, timeout=2):
    try:
        urllib.request.urlopen(test_url, timeout=timeout)
        return True
    except (urllib.error.URLError, urllib.error.HTTPError, Exception):
        return False


def check_jetson_network_connection(test_url=URL_TEST, timeout=2):
    python_code = f"""
import urllib.request
import urllib.error
try:
    urllib.request.urlopen('{test_url}', timeout={timeout})
    print('SUCCESS')
except:
    print('FAILED')
"""
    result = jetson_ssh_python_exec(python_code=python_code)
    return result.get("returncode") == 0 and "SUCCESS" in (result.get("stdout") or "")
