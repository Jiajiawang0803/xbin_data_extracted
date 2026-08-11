"""Extract one sensor topic from a Lixel/XGRIDS XBIN recording.

The decoder is the official ``lixel_bag.dll`` installed with LixelStudio 4.x.
This wrapper only supplies the MSVC x64 ABI objects needed to call its public
exports; it does not modify the source recording.
"""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import os
import struct
import sys
from pathlib import Path


DEFAULT_LIXEL_DIR = Path(r"C:\Program Files (x86)\LixelStudio")

SYM_BAG_CTOR = "??0LixelBag@lixel@@QEAA@XZ"
SYM_BAG_DTOR = "??1LixelBag@lixel@@QEAA@XZ"
SYM_BAG_OPEN = (
    "?open@LixelBag@lixel@@QEAA?AW4ErrCode@12@"
    "AEBV?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@"
    "AEBW4BagMode@12@0@Z"
)
SYM_BAG_CLOSE = "?close@LixelBag@lixel@@QEAAXXZ"
SYM_BAG_GET_VIEW = (
    "?getView@LixelBag@lixel@@QEAA?AVView@12@"
    "AEBV?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@_JH@Z"
)
SYM_VIEW_DTOR = "??1View@LixelBag@lixel@@QEAA@XZ"
SYM_UNPACKER_CTOR = (
    "??0DataUnpacker@lixel@@QEAA@PEAVLixelBag@1@"
    "AEBV?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@"
    "V?$function@$$A6A_N_K@Z@4@@Z"
)
SYM_UNPACKER_DTOR = "??1DataUnpacker@lixel@@UEAA@XZ"
SYM_EXPORT_DATA = (
    "?exportData@DataUnpacker@lixel@@QEAA_N"
    "AEBV?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@"
    "AEBVView@LixelBag@2@@Z"
)
SYM_FRAME_EXPORTER_CTOR = (
    "??0FrameExporter@lixel@@QEAA@PEAVLixelBag@1@"
    "AEBV?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@@Z"
)
SYM_FRAME_EXPORTER_DTOR = "??1FrameExporter@lixel@@UEAA@XZ"
SYM_EXPORT_FRAME = (
    "?exportFrame@FrameExporter@lixel@@QEAA_N"
    "AEBV?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@"
    "AEBVView@LixelBag@2@@Z"
)
SYM_EXPORT_IMAGE = (
    "?exportImage@FrameExporter@lixel@@QEAA_N"
    "AEBV?$basic_string@DU?$char_traits@D@std@@V?$allocator@D@2@@std@@"
    "AEBVView@LixelBag@2@0@Z"
)
SYM_UNPACK_LIDAR_DATA = (
    "?unpackLidarData@DataUnpacker@lixel@@AEAA_N"
    "PEAVFile@appkit@@AEBVView@LixelBag@2@@Z"
)

SUPPORTED_LIXEL_BAG_SHA256 = (
    "368F42BFB307D4716321C220714DACB5F848D70FBBBA83D86EABC7A0FFD2384C"
)


class BorrowedMsvcString:
    """Read-only MSVC x64 ``std::string`` passed to the vendor DLL by reference."""

    def __init__(self, value: str) -> None:
        encoded = value.encode("utf-8")
        self._storage = ctypes.create_string_buffer(encoded + b"\0")
        self._object = (ctypes.c_ubyte * 32)()
        address = ctypes.addressof(self._object)
        if len(encoded) <= 15:
            ctypes.memmove(address, encoded, len(encoded))
        else:
            ctypes.c_void_p.from_address(address).value = ctypes.addressof(self._storage)
        ctypes.c_size_t.from_address(address + 16).value = len(encoded)
        ctypes.c_size_t.from_address(address + 24).value = max(15, len(encoded))

    @property
    def ptr(self) -> ctypes.c_void_p:
        return ctypes.c_void_p(ctypes.addressof(self._object))


def bind(dll: ctypes.WinDLL, name: str, restype, argtypes):
    try:
        function = dll[name]
    except AttributeError as exc:
        raise RuntimeError(
            "The installed lixel_bag.dll is not ABI-compatible with this wrapper: "
            f"missing export {name}"
        ) from exc
    function.restype = restype
    function.argtypes = argtypes
    return function


def topic_output_files(output_dir: Path, topic_name: str) -> set[Path]:
    """Return only files owned by a topic, avoiding a full output-tree scan."""
    camera_directories = {
        "/camera_left/h264": "camera_0",
        "/camera_center/h264": "camera_1",
        "/camera_right/h264": "camera_2",
    }
    fixed_files = {
        "/gnss_data": "gnss.csv",
        "/raw_gnss_log": "raw_gnss_log.txt",
        "/raw_ntrip_log": "raw_ntrip_log.data",
        "/raw_ppk_log": "raw_ppk_log.data",
    }
    if topic_name == "/lidar":
        return set((output_dir / "lidar_points").glob("*.pcd"))
    if topic_name == "/config_file":
        return {p for p in (output_dir / "calib" / "config").glob("*") if p.is_file()}
    if topic_name in camera_directories:
        directory = output_dir / "images" / camera_directories[topic_name]
        return set(directory.glob("*.jpg"))
    if topic_name in fixed_files:
        path = output_dir / fixed_files[topic_name]
        return {path} if path.is_file() else set()
    return {p for p in output_dir.rglob("*") if p.is_file()}


class DirectBinaryPcdWriter:
    """Process-local fast path replacing Lixel's inlined ASCII frame writer."""

    _PATCH_BODY_OFFSET = 0x5CF
    # Resume at filename cleanup. The skipped header string remains in its
    # already-initialized empty state, and the skipped appkit::File object must
    # not be destroyed because it was never constructed.
    _RETURN_BODY_OFFSET = 0x98A
    _EXPECTED_PATCH_BYTES = bytes.fromhex("4c8b742468498b06488b5010")
    _CALLBACK = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)

    def __init__(self, dll: ctypes.WinDLL, install_dir: Path) -> None:
        import numpy as np

        self.np = np
        self.errors: list[str] = []
        self.frames_written = 0
        dll_path = install_dir / "lixel_bag.dll"
        digest = hashlib.sha256(dll_path.read_bytes()).hexdigest().upper()
        if digest != SUPPORTED_LIXEL_BAG_SHA256:
            raise RuntimeError(
                "Direct Binary PCD is only verified for LixelStudio 4.0.1.4 "
                f"(unexpected lixel_bag.dll SHA256 {digest})"
            )

        try:
            exported_function = dll[SYM_UNPACK_LIDAR_DATA]
        except AttributeError as exc:
            raise RuntimeError("Unable to locate Lixel's LiDAR unpacker") from exc
        thunk = ctypes.cast(exported_function, ctypes.c_void_p).value
        if thunk is None or ctypes.c_ubyte.from_address(thunk).value != 0xE9:
            raise RuntimeError("Unexpected LiDAR unpacker export thunk")
        relative = ctypes.c_int32.from_address(thunk + 1).value
        body = thunk + 5 + relative
        patch_address = body + self._PATCH_BODY_OFFSET
        return_address = body + self._RETURN_BODY_OFFSET
        actual = ctypes.string_at(patch_address, len(self._EXPECTED_PATCH_BYTES))
        if actual != self._EXPECTED_PATCH_BYTES:
            raise RuntimeError(
                "LiDAR writer machine-code verification failed; refusing to patch"
            )

        self.callback = self._CALLBACK(self._write_frame)
        callback_address = ctypes.cast(self.callback, ctypes.c_void_p).value
        if callback_address is None:
            raise RuntimeError("Unable to create Binary PCD callback")

        # r14 points at the current shared PointCloudFrame. The filename is the
        # already-constructed MSVC std::string at rsp+0x140. After the callback,
        # continue at the normal per-frame cleanup without constructing an
        # fstream or entering the vendor ASCII formatting loop.
        stub = (
            bytes.fromhex("4c8b742468")  # mov r14,[rsp+68h]
            + bytes.fromhex("4c89f1")  # mov rcx,r14
            + bytes.fromhex("488d942440010000")  # lea rdx,[rsp+140h]
            + bytes.fromhex("4883ec20")  # shadow space
            + b"\x48\xB8"
            + struct.pack("<Q", callback_address)
            + bytes.fromhex("ffd0")  # call rax
            + bytes.fromhex("4883c420")
            + bytes.fromhex("4531ff")  # xor r15d,r15d
            + bytes.fromhex("4c8bac2488000000")  # mov r13,[rsp+88h]
            + b"\x48\xB8"
            + struct.pack("<Q", return_address)
            + bytes.fromhex("ffe0")  # jmp rax
        )

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        virtual_alloc = kernel32.VirtualAlloc
        virtual_alloc.argtypes = [
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_ulong,
            ctypes.c_ulong,
        ]
        virtual_alloc.restype = ctypes.c_void_p
        self.stub_address = virtual_alloc(None, len(stub), 0x3000, 0x40)
        if not self.stub_address:
            raise ctypes.WinError(ctypes.get_last_error())
        ctypes.memmove(self.stub_address, stub, len(stub))

        jump = b"\x48\xB8" + struct.pack("<Q", int(self.stub_address)) + b"\xFF\xE0"
        virtual_protect = kernel32.VirtualProtect
        virtual_protect.argtypes = [
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong),
        ]
        virtual_protect.restype = ctypes.c_bool
        old_protection = ctypes.c_ulong()
        if not virtual_protect(
            ctypes.c_void_p(patch_address), len(jump), 0x40, ctypes.byref(old_protection)
        ):
            raise ctypes.WinError(ctypes.get_last_error())
        try:
            ctypes.memmove(patch_address, jump, len(jump))
            flush = kernel32.FlushInstructionCache
            flush.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t]
            flush.restype = ctypes.c_bool
            get_process = kernel32.GetCurrentProcess
            get_process.restype = ctypes.c_void_p
            if not flush(get_process(), ctypes.c_void_p(patch_address), len(jump)):
                raise ctypes.WinError(ctypes.get_last_error())
        finally:
            restored = ctypes.c_ulong()
            virtual_protect(
                ctypes.c_void_p(patch_address),
                len(jump),
                old_protection.value,
                ctypes.byref(restored),
            )

    @staticmethod
    def _msvc_string(address: int) -> str:
        length = ctypes.c_size_t.from_address(address + 16).value
        if length <= 15:
            data_address = address
        else:
            data_address = ctypes.c_void_p.from_address(address).value
            if data_address is None:
                raise RuntimeError("Null MSVC string data")
        return ctypes.string_at(data_address, length).decode("utf-8")

    def _write_frame(self, frame_pointer: int, filename_pointer: int) -> None:
        try:
            level_1 = ctypes.c_void_p.from_address(frame_pointer).value
            if level_1 is None:
                raise RuntimeError("Null PointCloudFrame")
            level_2 = ctypes.c_void_p.from_address(level_1 + 0x10).value
            if level_2 is None:
                raise RuntimeError("Null LiDAR data")
            cloud = ctypes.c_void_p.from_address(level_2 + 0x10).value
            if cloud is None:
                raise RuntimeError("Null PCL cloud")
            begin = ctypes.c_void_p.from_address(cloud + 0x30).value or 0
            end = ctypes.c_void_p.from_address(cloud + 0x38).value or 0
            byte_count = end - begin
            if byte_count < 0 or byte_count % 0x50:
                raise RuntimeError(f"Invalid PointIRT vector size: {byte_count}")
            point_count = byte_count // 0x50
            filename = Path(self._msvc_string(filename_pointer))

            source_dtype = self.np.dtype(
                {
                    "names": ("x", "y", "z", "intensity", "timestamp", "ring"),
                    "formats": ("<f4", "<f4", "<f4", "<f4", "<f8", "<u2"),
                    "offsets": (0x00, 0x04, 0x08, 0x24, 0x28, 0x40),
                    "itemsize": 0x50,
                }
            )
            output_dtype = self.np.dtype(
                {
                    "names": (
                        "x",
                        "y",
                        "z",
                        "normal_x",
                        "normal_y",
                        "normal_z",
                        "intensity",
                        "timestamp",
                        "ring",
                    ),
                    "formats": ("<f4",) * 7 + ("<f8", "<u2"),
                    "offsets": (0, 4, 8, 12, 16, 20, 24, 28, 36),
                    "itemsize": 38,
                }
            )
            memory = (ctypes.c_ubyte * byte_count).from_address(begin)
            source = self.np.frombuffer(memory, dtype=source_dtype, count=point_count)
            output = self.np.zeros(point_count, dtype=output_dtype)
            for field in ("x", "y", "z", "intensity", "timestamp", "ring"):
                output[field] = source[field]

            header = (
                "# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION 0.7\n"
                "FIELDS x y z normal_x normal_y normal_z intensity timestamp ring\n"
                "SIZE 4 4 4 4 4 4 4 8 2\n"
                "TYPE F F F F F F F F U\n"
                "COUNT 1 1 1 1 1 1 1 1 1\n"
                f"WIDTH {point_count}\n"
                "HEIGHT 1\n"
                "VIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {point_count}\n"
                "DATA binary\n"
            ).encode("ascii")
            filename.parent.mkdir(parents=True, exist_ok=True)
            with filename.open("wb", buffering=0) as handle:
                handle.write(header)
                handle.write(output.tobytes(order="C"))
            self.frames_written += 1
        except BaseException as exc:
            self.errors.append(f"{type(exc).__name__}: {exc}")


def enable_direct_binary_pcd(
    dll: ctypes.WinDLL, install_dir: Path
) -> DirectBinaryPcdWriter:
    """Enable verified direct Binary PCD serialization for this child process."""
    return DirectBinaryPcdWriter(dll, install_dir)


class LixelDecoder:
    def __init__(self, install_dir: Path) -> None:
        dll_path = install_dir / "lixel_bag.dll"
        if not dll_path.is_file():
            raise FileNotFoundError(f"Lixel decoder not found: {dll_path}")
        self._dll_dir_handle = os.add_dll_directory(str(install_dir))
        self.dll = ctypes.WinDLL(str(dll_path))

        self.bag_ctor = bind(self.dll, SYM_BAG_CTOR, ctypes.c_void_p, [ctypes.c_void_p])
        self.bag_dtor = bind(self.dll, SYM_BAG_DTOR, None, [ctypes.c_void_p])
        self.bag_open = bind(
            self.dll,
            SYM_BAG_OPEN,
            ctypes.c_int,
            [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p],
        )
        self.bag_close = bind(self.dll, SYM_BAG_CLOSE, None, [ctypes.c_void_p])
        # MSVC member UDT return ABI: this, hidden return buffer, regular args.
        self.bag_get_view = bind(
            self.dll,
            SYM_BAG_GET_VIEW,
            ctypes.c_void_p,
            [
                ctypes.c_void_p,
                ctypes.c_void_p,
                ctypes.c_void_p,
                ctypes.c_int64,
                ctypes.c_int,
            ],
        )
        self.view_dtor = bind(self.dll, SYM_VIEW_DTOR, None, [ctypes.c_void_p])
        self.unpacker_ctor = bind(
            self.dll,
            SYM_UNPACKER_CTOR,
            ctypes.c_void_p,
            [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p],
        )
        self.unpacker_dtor = bind(
            self.dll, SYM_UNPACKER_DTOR, None, [ctypes.c_void_p]
        )
        self.export_data = bind(
            self.dll,
            SYM_EXPORT_DATA,
            ctypes.c_bool,
            [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p],
        )
        self.frame_exporter_ctor = bind(
            self.dll,
            SYM_FRAME_EXPORTER_CTOR,
            ctypes.c_void_p,
            [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p],
        )
        self.frame_exporter_dtor = bind(
            self.dll, SYM_FRAME_EXPORTER_DTOR, None, [ctypes.c_void_p]
        )
        self.export_frame = bind(
            self.dll,
            SYM_EXPORT_FRAME,
            ctypes.c_bool,
            [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p],
        )
        self.export_image = bind(
            self.dll,
            SYM_EXPORT_IMAGE,
            ctypes.c_bool,
            [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p],
        )

    def extract(
        self,
        xbin: Path,
        output_dir: Path,
        start_us: int,
        duration_s: int,
        topic_name: str = "/lidar",
        raw_frame: bool = False,
        image_format: str | None = None,
    ) -> tuple[int, int]:
        # These buffers are deliberately oversized opaque storage for vendor C++
        # objects. Their constructors/destructors remain responsible for members.
        bag_storage = (ctypes.c_ubyte * 4096)()
        unpacker_storage = (ctypes.c_ubyte * 16384)()
        frame_exporter_storage = (ctypes.c_ubyte * 16384)()
        view_storage = (ctypes.c_ubyte * 256)()
        empty_function = (ctypes.c_ubyte * 64)()  # default/empty std::function

        bag = ctypes.c_void_p(ctypes.addressof(bag_storage))
        unpacker = ctypes.c_void_p(ctypes.addressof(unpacker_storage))
        frame_exporter = ctypes.c_void_p(ctypes.addressof(frame_exporter_storage))
        view = ctypes.c_void_p(ctypes.addressof(view_storage))
        xbin_string = BorrowedMsvcString(str(xbin.resolve()))
        output_string = BorrowedMsvcString(str(output_dir.resolve()))
        empty_string = BorrowedMsvcString("")
        topic = BorrowedMsvcString(topic_name)
        image_format_string = BorrowedMsvcString(image_format or "")
        mode = ctypes.c_int(0)  # read mode

        bag_constructed = False
        bag_opened = False
        view_constructed = False
        unpacker_constructed = False
        frame_exporter_constructed = False
        try:
            self.bag_ctor(bag)
            bag_constructed = True
            result = self.bag_open(
                bag, xbin_string.ptr, ctypes.byref(mode), empty_string.ptr
            )
            if result != 0:
                raise RuntimeError(f"LixelBag::open failed with error code {result}")
            bag_opened = True

            before = topic_output_files(output_dir, topic_name)
            self.bag_get_view(bag, view, topic.ptr, start_us, duration_s)
            view_constructed = True
            begin = ctypes.c_void_p.from_address(ctypes.addressof(view_storage)).value or 0
            end = ctypes.c_void_p.from_address(ctypes.addressof(view_storage) + 8).value or 0
            # Each vendor MsgIndex is 0x70 bytes in LixelStudio 4.0.1.4.
            selected_indexes = (end - begin) // 0x70 if end >= begin else 0
            print(f"selected_index_chunks={selected_indexes}")
            if selected_indexes == 0:
                raise RuntimeError(
                    f"No {topic_name} messages matched the requested range"
                )

            if raw_frame:
                self.frame_exporter_ctor(frame_exporter, bag, output_string.ptr)
                frame_exporter_constructed = True
                if image_format:
                    exported = self.export_image(
                        frame_exporter, topic.ptr, view, image_format_string.ptr
                    )
                    exporter_name = "exportImage"
                else:
                    exported = self.export_frame(frame_exporter, topic.ptr, view)
                    exporter_name = "exportFrame"
                if not exported:
                    raise RuntimeError(
                        f"FrameExporter::{exporter_name}({topic_name}) returned false"
                    )
            else:
                self.unpacker_ctor(
                    unpacker,
                    bag,
                    output_string.ptr,
                    ctypes.c_void_p(ctypes.addressof(empty_function)),
                )
                unpacker_constructed = True
                if not self.export_data(unpacker, topic.ptr, view):
                    raise RuntimeError(
                        f"DataUnpacker::exportData({topic_name}) returned false"
                    )
            after = topic_output_files(output_dir, topic_name)
            return selected_indexes, len(after - before)
        finally:
            if frame_exporter_constructed:
                self.frame_exporter_dtor(frame_exporter)
            if unpacker_constructed:
                self.unpacker_dtor(unpacker)
            if view_constructed:
                self.view_dtor(view)
            if bag_opened:
                self.bag_close(bag)
            if bag_constructed:
                self.bag_dtor(bag)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract one timestamped sensor topic from a Lixel XBIN"
    )
    parser.add_argument("xbin", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--start-us",
        type=int,
        default=0,
        help="Start timestamp in integer microseconds; 0 starts at the first frame",
    )
    parser.add_argument(
        "--duration-s",
        type=int,
        default=1,
        help="Duration to export in seconds; 0 exports all remaining frames",
    )
    parser.add_argument("--lixel-dir", type=Path, default=DEFAULT_LIXEL_DIR)
    parser.add_argument(
        "--topic",
        default="/lidar",
        help="XBIN topic to export (default: /lidar)",
    )
    parser.add_argument(
        "--raw-frame",
        action="store_true",
        help="Use FrameExporter (needed for raw H.264 camera streams)",
    )
    parser.add_argument(
        "--image-format",
        choices=("h264", "jpeg"),
        help="Camera export format; implies --raw-frame",
    )
    parser.add_argument(
        "--binary-pcd",
        action="store_true",
        help="Write /lidar directly as standard DATA binary PCD",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.xbin.is_file():
        print(f"XBIN not found: {args.xbin}", file=sys.stderr)
        return 2
    if args.duration_s < 0:
        print("--duration-s must be >= 0", file=sys.stderr)
        return 2
    args.output.mkdir(parents=True, exist_ok=True)
    try:
        decoder = LixelDecoder(args.lixel_dir)
        binary_writer = None
        if args.binary_pcd:
            if args.topic != "/lidar":
                raise ValueError("--binary-pcd can only be used with --topic /lidar")
            binary_writer = enable_direct_binary_pcd(decoder.dll, args.lixel_dir)
        selected_indexes, new_frames = decoder.extract(
            args.xbin,
            args.output,
            args.start_us,
            args.duration_s,
            args.topic,
            args.raw_frame or bool(args.image_format),
            args.image_format,
        )
        if binary_writer is not None and binary_writer.errors:
            raise RuntimeError(
                "Binary PCD writer failed: " + "; ".join(binary_writer.errors[:3])
            )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(
        "Extraction completed: "
        f"{selected_indexes} internal index chunk(s), {new_frames} new file(s)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
