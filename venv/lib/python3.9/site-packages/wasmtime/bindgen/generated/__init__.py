from .imports import BindgenImports
from .intrinsics import _decode_utf8, _encode_utf8, _list_canon_lift, _list_canon_lower, _load
from .types import Err, Ok, Result
import ctypes
import os
from typing import List, Tuple, cast
import wasmtime

class Bindgen:
    
    def __init__(self, store: wasmtime.Store, import_object: BindgenImports) -> None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bindgen.core2.wasm')
        module = wasmtime.Module.from_file(store.engine, path)
        instance0 = wasmtime.Instance(store, module, []).exports(store)
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bindgen.core0.wasm')
        module = wasmtime.Module.from_file(store.engine, path)
        instance1 = wasmtime.Instance(store, module, [
            instance0["2"],
            instance0["3"],
            instance0["4"],
            instance0["5"],
            instance0["6"],
        ]).exports(store)
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bindgen.core1.wasm')
        module = wasmtime.Module.from_file(store.engine, path)
        instance2 = wasmtime.Instance(store, module, [
            instance1["memory"],
            instance0["0"],
            instance0["1"],
        ]).exports(store)
        core_memory0 = instance1["memory"]
        assert(isinstance(core_memory0, wasmtime.Memory))
        self._core_memory0 = core_memory0
        def lowering0_callee(caller: wasmtime.Caller, arg0: int, arg1: int) -> None:
            ptr = arg0
            len0 = arg1
            list = cast(bytes, _list_canon_lift(ptr, len0, 1, ctypes.c_uint8, self._core_memory0, caller))
            import_object.python.print(list)
        lowering0_ty = wasmtime.FuncType([wasmtime.ValType.i32(), wasmtime.ValType.i32(), ], [])
        lowering0 = wasmtime.Func(store, lowering0_ty, lowering0_callee, access_caller = True)
        def lowering1_callee(caller: wasmtime.Caller, arg0: int, arg1: int) -> None:
            ptr = arg0
            len0 = arg1
            list = cast(bytes, _list_canon_lift(ptr, len0, 1, ctypes.c_uint8, self._core_memory0, caller))
            import_object.python.eprint(list)
        lowering1_ty = wasmtime.FuncType([wasmtime.ValType.i32(), wasmtime.ValType.i32(), ], [])
        lowering1 = wasmtime.Func(store, lowering1_ty, lowering1_callee, access_caller = True)
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bindgen.core3.wasm')
        module = wasmtime.Module.from_file(store.engine, path)
        instance3 = wasmtime.Instance(store, module, [
            lowering0,
            lowering1,
            instance2["fd_write"],
            instance2["random_get"],
            instance2["environ_get"],
            instance2["environ_sizes_get"],
            instance2["proc_exit"],
            instance0["$imports"],
        ]).exports(store)
        realloc0 = instance1["cabi_realloc"]
        assert(isinstance(realloc0, wasmtime.Func))
        self._realloc0 = realloc0
        post_return0 = instance1["cabi_post_generate"]
        assert(isinstance(post_return0, wasmtime.Func))
        self._post_return0 = post_return0
        lift_callee0 = instance1["generate"]
        assert(isinstance(lift_callee0, wasmtime.Func))
        self.lift_callee0 = lift_callee0
    def generate(self, caller: wasmtime.Store, name: str, wit: bytes) -> Result[List[Tuple[str, bytes]], str]:
        ptr, len0 = _encode_utf8(name, self._realloc0, self._core_memory0, caller)
        ptr1, len2 = _list_canon_lower(wit, ctypes.c_uint8, 1, 1, self._realloc0, self._core_memory0, caller)
        ret = self.lift_callee0(caller, ptr, len0, ptr1, len2)
        assert(isinstance(ret, int))
        load = _load(ctypes.c_uint8, self._core_memory0, caller, ret, 0)
        expected: Result[List[Tuple[str, bytes]], str]
        if load == 0:
            load3 = _load(ctypes.c_int32, self._core_memory0, caller, ret, 4)
            load4 = _load(ctypes.c_int32, self._core_memory0, caller, ret, 8)
            ptr15 = load3
            len16 = load4
            result: List[Tuple[str, bytes]] = []
            for i17 in range(0, len16):
                base5 = ptr15 + i17 * 16
                load6 = _load(ctypes.c_int32, self._core_memory0, caller, base5, 0)
                load7 = _load(ctypes.c_int32, self._core_memory0, caller, base5, 4)
                ptr8 = load6
                len9 = load7
                list = _decode_utf8(self._core_memory0, caller, ptr8, len9)
                load10 = _load(ctypes.c_int32, self._core_memory0, caller, base5, 8)
                load11 = _load(ctypes.c_int32, self._core_memory0, caller, base5, 12)
                ptr12 = load10
                len13 = load11
                list14 = cast(bytes, _list_canon_lift(ptr12, len13, 1, ctypes.c_uint8, self._core_memory0, caller))
                result.append((list, list14,))
            expected = Ok(result)
        elif load == 1:
            load18 = _load(ctypes.c_int32, self._core_memory0, caller, ret, 4)
            load19 = _load(ctypes.c_int32, self._core_memory0, caller, ret, 8)
            ptr20 = load18
            len21 = load19
            list22 = _decode_utf8(self._core_memory0, caller, ptr20, len21)
            expected = Err(list22)
        else:
            raise TypeError("invalid variant discriminant for expected")
        self._post_return0(caller, ret)
        return expected
