#!/usr/bin/env python3
import os
import shlex
import subprocess


def _try_ros2_casadi_vendor():
    try:
        from ament_index_python.packages import get_package_prefix  # type: ignore

        prefix = get_package_prefix("casadi_vendor")
    except Exception:
        return None

    include_dir = os.path.join(prefix, "include")
    lib_dir = os.path.join(prefix, "lib")
    if os.path.isdir(include_dir) and os.path.isdir(lib_dir):
        return {"include_dir": include_dir, "lib_dir": lib_dir}
    return None


def _run_pkg_config(package: str):
    try:
        cflags = subprocess.check_output(["pkg-config", "--cflags", package], text=True).strip()
        libs = subprocess.check_output(["pkg-config", "--libs", package], text=True).strip()
        return {"cflags": shlex.split(cflags), "libs": shlex.split(libs)}
    except Exception:
        return None


def _resolve_casadi_paths():
    # Highest priority: explicit env override
    include_dir = os.environ.get("CASADI_INCLUDE_DIR")
    lib_dir = os.environ.get("CASADI_LIB_DIR")
    if include_dir and lib_dir:
        return {"include_dir": include_dir, "lib_dir": lib_dir}

    # Next: ROS2 casadi_vendor (if present)
    ros_vendor = _try_ros2_casadi_vendor()
    if ros_vendor:
        return ros_vendor

    # Next: pkg-config (common after system install)
    pc = _run_pkg_config("casadi")
    if pc:
        inc = next((f[2:] for f in pc["cflags"] if f.startswith("-I")), None)
        lib = next((f[2:] for f in pc["libs"] if f.startswith("-L")), None)
        if inc and lib:
            return {"include_dir": inc, "lib_dir": lib, "extra_libs": pc["libs"]}

    # Fallback: typical default prefix for `sudo make install`
    for prefix in ("/usr/local", "/usr"):
        cand_inc = os.path.join(prefix, "include")
        cand_lib = os.path.join(prefix, "lib")
        if os.path.isdir(cand_inc) and os.path.isdir(cand_lib):
            return {"include_dir": cand_inc, "lib_dir": cand_lib}

    return None


def _resolve_ipopt_include_flags():
    # Allow explicit override
    inc = os.environ.get("IPOPT_INCLUDE_DIR")
    if inc and os.path.isdir(inc):
        return [f"-I{inc}"]

    def _shim_coin_or_from_coin(include_root: str):
        """Some distros install Ipopt headers under `coin/` (e.g. /usr/include/coin/IpStdCInterface.h)
        while generated code includes `<coin-or/...>`. Create a local shim:
          <shim_root>/coin-or -> <include_root>/coin
        and then use -I<shim_root>.
        """
        coin_dir = os.path.join(include_root, "coin")
        if not os.path.isfile(os.path.join(coin_dir, "IpStdCInterface.h")):
            return None

        shim_root = os.path.join(script_dir, ".include_shims")
        shim_link = os.path.join(shim_root, "coin-or")
        os.makedirs(shim_root, exist_ok=True)
        try:
            if os.path.islink(shim_link) or os.path.exists(shim_link):
                return shim_root
            os.symlink(coin_dir, shim_link)
            return shim_root
        except OSError:
            return None

    # If headers are installed, prefer an include dir that satisfies:
    #   include_dir/coin-or/IpStdCInterface.h exists
    for base in ("/usr/include", "/usr/local/include"):
        hdr = os.path.join(base, "coin-or", "IpStdCInterface.h")
        if os.path.isfile(hdr):
            return [f"-I{base}"]

    # If headers exist under `coin/`, create a shim so `<coin-or/...>` resolves.
    for base in ("/usr/include", "/usr/local/include"):
        shim_root = _shim_coin_or_from_coin(base)
        if shim_root:
            return [f"-I{shim_root}", f"-I{base}"]

    # pkg-config if available (may return -I/usr/include/coin which won't work for <coin-or/...>)
    pc = _run_pkg_config("ipopt")
    if pc:
        inc_flags = [f for f in pc["cflags"] if f.startswith("-I")]
        fixed = []
        for f in inc_flags:
            p = f[2:]
            fixed.append(f)
            parent = os.path.dirname(p)
            if os.path.isfile(os.path.join(parent, "coin-or", "IpStdCInterface.h")):
                fixed.append(f"-I{parent}")
        return list(dict.fromkeys(fixed))  # de-dup preserving order

    return []


def _include_flags_have_header(include_flags, rel_header_path: str) -> bool:
    for f in include_flags:
        if not f.startswith("-I"):
            continue
        base = f[2:]
        if os.path.isfile(os.path.join(base, rel_header_path)):
            return True
    return False


script_dir = os.path.dirname(os.path.realpath(__file__))
# walk through the directory and find all the .c files
c_files = []
for root, _dirs, files in os.walk(script_dir):
    for file in files:
        if file.endswith(".c"):
            c_files.append(os.path.join(root, file))


def compile_c_files(c_file):
    casadi = _resolve_casadi_paths()
    if not casadi:
        raise SystemExit(
            "无法定位 CasADi 安装位置。请设置环境变量 CASADI_INCLUDE_DIR 和 CASADI_LIB_DIR,"
            "或确保系统存在 `pkg-config casadi`。"
        )

    casadi_include_path = casadi["include_dir"]
    casadi_lib_path = casadi["lib_dir"]
    ipopt_inc_flags = _resolve_ipopt_include_flags()
    if not ipopt_inc_flags or not _include_flags_have_header(
        ipopt_inc_flags, os.path.join("coin-or", "IpStdCInterface.h")
    ):
        raise SystemExit(
            "找不到 IPOPT 头文件 `coin-or/IpStdCInterface.h`, 无法编译生成的 .c 文件。\n"
            "请安装 ipopt 开发包 (Ubuntu 常用: `sudo apt-get install -y coinor-libipopt-dev`),\n"
            "或设置环境变量 IPOPT_INCLUDE_DIR 指向包含 `coin-or/IpStdCInterface.h` 的 include 根目录。"
        )

    compile_flags = [
        "-std=c99",
        "-fPIC",
        "-shared",
        "-O2",
        "-include",
        "stdbool.h",
        "-Dipindex=Index",
        "-Dipnumber=Number",
        *ipopt_inc_flags,
        f"-I{casadi_include_path}",
        f"-L{casadi_lib_path}",
        "-lm",
        "-lipopt",
        "-lcasadi",
        f"-Wl,-rpath,{casadi_lib_path}",
    ]

    output_file = c_file.replace(".c", ".so")
    compile_command = ["gcc", c_file, "-o", output_file, *compile_flags]
    print(f"Compiling {c_file} to {output_file}")
    print("-" * 50)
    print("Compile command:", " ".join(shlex.quote(x) for x in compile_command))

    subprocess.check_call(compile_command)
    print("")


# display the list of files
print("C files to be compiled:")
for file in c_files:
    print(file)
    compile_c_files(file)
