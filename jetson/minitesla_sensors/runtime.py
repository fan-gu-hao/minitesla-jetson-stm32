"""Linux 程序與 UART 資源的排他持有。

SingleInstance 應包住整個主程式生命週期，在建立子行程與開啟裝置前取得。
固定路徑的 flock 由核心持有，最後一個描述元關閉後自動釋放；鎖檔保留，
不以檔案存在與否判斷執行狀態。它協調使用相同本機檔案的新版主程式，
不是阻擋所有舊程式的權限機制。

UART 同時使用 pySerial 的 advisory flock 與 Linux TIOCEXCL。TIOCEXCL
阻止後續一般 open，但不驅逐原先已開啟的描述元，具 CAP_SYS_ADMIN 的程序
也可能繞過。因此其他舊串口測試程式仍須先停止。
"""

import errno
import fcntl
import os
import stat
import sys
import termios


DEFAULT_INSTANCE_PATH = "/tmp/minitesla-vehicle.lock"


class SingleInstanceError(RuntimeError):
    """另一份車端主程式已占用相同的核心資源。"""


def _require_linux():
    if not sys.platform.startswith("linux"):
        raise RuntimeError("車端資源排他保護只支援 Linux／Jetson。")


class SingleInstance:
    """跨工作資料夾、一般使用者／sudo 的單一實例持有器。

    path 只供不同應用或測試隔離使用；正式主程式請使用固定預設路徑。
    不寫入 PID、不截斷、不移除檔案，也不更改既有檔案權限。
    描述元不會跨 exec 繼承。使用 multiprocessing 時仍應採用 spawn；
    fork 本身會複製描述元。若 fork 子程序誤用此物件，PID 檢查會拒絕操作，
    並只關閉子程序自己的描述元副本，不送 LOCK_UN 影響父程序。
    """

    def __init__(self, path=DEFAULT_INSTANCE_PATH):
        self._path = os.fspath(path)
        if not isinstance(self._path, str) or not self._path or "\x00" in self._path:
            raise ValueError("實例鎖路徑必須是不含 NUL 的非空字串。")
        self._fd = None
        self._owner_pid = None

    def _reject_inherited_handle(self):
        if self._fd is not None and self._owner_pid != os.getpid():
            # close 只減少本程序的參照；不可送 LOCK_UN 解除父程序的 flock。
            os.close(self._fd)
            self._fd = None
            self._owner_pid = None
            raise RuntimeError("不可在另一個行程操作繼承的實例鎖；請使用 spawn。")

    def acquire(self):
        _require_linux()
        self._reject_inherited_handle()
        if self._fd is not None:
            raise RuntimeError("同一實例鎖物件不能重複取得；請勿巢狀使用。")
        flags = os.O_NOFOLLOW | os.O_CLOEXEC | os.O_NONBLOCK
        created = False
        try:
            held = os.open(self._path, os.O_RDWR | os.O_CREAT | os.O_EXCL | flags, 0o666)
            created = True
        except FileExistsError:
            # Linux flock 排他鎖可由唯讀 fd 取得。避免 sudo 先建立檔案後，
            # 一般使用者因檔案擁有者或 protected_regular 規則而不能重開。
            held = os.open(self._path, os.O_RDONLY | flags)
        try:
            if not stat.S_ISREG(os.fstat(held).st_mode):
                raise RuntimeError("實例鎖路徑必須是一般檔案；已拒絕啟動。")
            if created:
                os.fchmod(held, 0o666)
            os.set_inheritable(held, False)
            fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            os.close(held)
            if exc.errno in (errno.EAGAIN, errno.EACCES):
                raise SingleInstanceError(
                    "另一份智慧車車端主程式仍在執行；請先正常關閉該程式。"
                ) from exc
            raise
        except BaseException:
            os.close(held)
            raise
        self._fd = held
        self._owner_pid = os.getpid()
        return self

    def release(self):
        self._reject_inherited_handle()
        held = self._fd
        self._fd = None
        self._owner_pid = None
        if held is not None:
            os.close(held)

    def __enter__(self):
        return self.acquire()

    def __exit__(self, exc_type, exc_value, traceback):
        self.release()
        return False


def open_exclusive_uart(device, baud, write_timeout=0.02):
    """開啟非阻塞讀取、限時寫入且排他的 UART；失敗即關閉自己的 fd。

    pySerial 已是車端原本依賴。exclusive=True 的 flock 只保護遵循相同
    協定的開啟者；TIOCEXCL 再阻擋後續一般 open。它不處理既有舊 fd。
    """
    _require_linux()
    import serial

    ser = serial.Serial(
        device, baud, timeout=0, write_timeout=write_timeout, exclusive=True
    )
    exclusive_acquired = False
    try:
        fd = ser.fileno()
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive_acquired = True
        ser._minitesla_exclusive_owner = (os.getpid(), fd)
        return ser
    except BaseException:
        # TIOCEXCL 未成功時，不得解除其他持有者設定的排他狀態。
        try:
            if exclusive_acquired:
                fcntl.ioctl(ser.fileno(), termios.TIOCNXCL)
        finally:
            ser.close()
        raise


def close_exclusive_uart(ser):
    """解除本程序成功取得的 UART 排他狀態，並確保描述元被關閉。

    可重複呼叫。非本程序持有的物件只關閉本地 fd 副本，不送 TIOCNXCL。
    釋放 ioctl 若失敗會在關閉 fd 後回報例外，不靜默宣稱已解除。
    """
    if ser is None:
        return
    owner = getattr(ser, "_minitesla_exclusive_owner", None)
    if owner is not None:
        del ser._minitesla_exclusive_owner
    try:
        if owner is not None and owner[0] == os.getpid() and ser.is_open:
            fd = ser.fileno()
            if fd == owner[1]:
                fcntl.ioctl(fd, termios.TIOCNXCL)
    finally:
        ser.close()
