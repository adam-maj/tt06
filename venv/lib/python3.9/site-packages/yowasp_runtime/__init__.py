import os
import sys
import shutil
import tempfile
import wasmtime
import pathlib
import hashlib
import appdirs
try:
    from importlib import resources as importlib_resources
    importlib_resources.files
except (ImportError, AttributeError):
    import importlib_resources


def run_wasm(__package__, wasm_filename, *, resources=[], argv):
    # load the WebAssembly application
    module_binary = importlib_resources.read_binary(__package__, wasm_filename)
    module_digest = hashlib.sha1(module_binary).hexdigest()

    wasi_cfg = wasmtime.WasiConfig()

    # inherit standard I/O handles
    wasi_cfg.inherit_stdin()
    wasi_cfg.inherit_stdout()
    wasi_cfg.inherit_stderr()

    # use provided argv
    wasi_cfg.argv = argv

    if "YOWASP_MOUNT" in os.environ:
        # preopens for explicitly specified paths; if YOWASP_MOUNT is specified, the YoWASP
        # runtime can be used as a security boundary (subject to your confidence in wasmtime)
        for mount_entry in os.environ["YOWASP_MOUNT"].split(":"):
            if mount_entry.count("=") < 1:
                print("Invalid entry {!r} in YOWASP_MOUNT environment variable: "
                      "must be separated by '='"
                      .format(mount_entry), file=sys.stderr)
                return 64 # BSD sysexits(3) EX_USAGE

            mountpoint, target = mount_entry.split("=", 1)
            if not os.path.isdir(target):
                print("Invalid entry {!r} in YOWASP_MOUNT environment variable: "
                      "OS path {!r} must exist and be a directory"
                      .format(mount_entry, target), file=sys.stderr)
                return 64 # BSD sysexits(3) EX_USAGE
            if not mountpoint.startswith("/"):
                print("Invalid entry {!r} in YOWASP_MOUNT environment variable: "
                      "mountpoint {!r} must be an absolute path"
                      .format(mount_entry, mountpoint), file=sys.stderr)
                return 64 # BSD sysexits(3) EX_USAGE

            wasi_cfg.preopen_dir(target, mountpoint)

    else:
        # preopens for absolute paths
        if os.name == "nt":
            for letter in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ":
                wasi_cfg.preopen_dir(letter + ":\\", letter + ":")
        else:
            # can't do this for files, but no one's going to use yowasp on files in / anyway
            for path in os.listdir("/"):
                if os.path.isdir("/" + path):
                    wasi_cfg.preopen_dir("/" + path, "/" + path)

        # preopens for relative paths
        wasi_cfg.preopen_dir(".", ".")
        for level in range(len(pathlib.Path().cwd().parts)):
            wasi_cfg.preopen_dir(str(pathlib.Path("").joinpath(*[".."] * level)),
                                 "/".join([".."] * level))

    # preopens for package resources; these are necessary for package functionality
    # and take priority over implicit or explicit OS mounts
    for resource in resources:
        wasi_cfg.preopen_dir(str(importlib_resources.files(__package__) / resource), 
                             "/" + resource)
    
    # preopen for temporary directory; this is necessary for package functionality
    # and takes priority over implicit or explicit OS mounts
    temp_dirname = tempfile.mkdtemp(prefix="yowasp_")
    wasi_cfg.preopen_dir(temp_dirname, "/tmp")

    try:
        # compute path to cache
        default_cache_path = appdirs.user_cache_dir("YoWASP", appauthor=False)
        cache_path = pathlib.Path(os.environ.get("YOWASP_CACHE_DIR", default_cache_path))
        cache_filename = (cache_path / __package__ / wasm_filename / module_digest)

        # compile WebAssembly to machine code, or load cached
        engine = wasmtime.Engine()
        module = None
        if cache_filename.exists():
            try:
                module = wasmtime.Module.deserialize_file(engine, str(cache_filename))
            except wasmtime.WasmtimeError:
                pass
        if module is None:
            print("Preparing to run {}. This might take a while..."
                  .format(argv[0]), file=sys.stderr)
            module = wasmtime.Module(engine, module_binary)
            cache_filename.parent.mkdir(parents=True, exist_ok=True)
            with cache_filename.open("wb") as cache_file:
                cache_file.write(module.serialize())

        # run compiled code
        linker = wasmtime.Linker(engine)
        linker.define_wasi()
        store = wasmtime.Store(engine)
        store.set_wasi(wasi_cfg)
        app = linker.instantiate(store, module)
        linker.define_instance(store, "app", app)
        try:
            app.exports(store)["_start"](store)
            return 0
        except wasmtime.ExitTrap as trap:
            return trap.code

    finally:
        # clean up temporary directory; on Windows this can cause errors in certain cases
        # and there isn't a solution much better than just ignoring them
        shutil.rmtree(temp_dirname, ignore_errors=True)
