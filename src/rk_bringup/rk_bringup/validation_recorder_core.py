"""动态验收 recorder 的有界队列与异步 JSONL writer。"""

import json
import os
from pathlib import Path
import queue
import threading
import time


class BoundedJsonlWriter:
    """callback 只 put_nowait；独立线程批量写盘并在关闭时持久化统计。"""

    def __init__(
        self, path, *, queue_size=2048, batch_size=64,
        flush_interval_sec=0.1, writer_delay_sec=0.0,
    ):
        if queue_size <= 0 or batch_size <= 0 or flush_interval_sec <= 0.0:
            raise ValueError(
                'recorder queue/batch/flush limits must be positive'
            )
        self.path = Path(path)
        self.queue = queue.Queue(maxsize=int(queue_size))
        self.batch_size = int(batch_size)
        self.flush_interval_sec = float(flush_interval_sec)
        self.writer_delay_sec = max(0.0, float(writer_delay_sec))
        self.enqueued = 0
        self.written = 0
        self.dropped = 0
        self.max_queue_depth = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._error = None

    def start(self):
        """启动唯一 writer thread；重复启动视为编程错误。"""
        if self._thread is not None:
            raise RuntimeError('recorder writer already started')
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._thread = threading.Thread(
            target=self._run, name='validation-jsonl-writer', daemon=True,
        )
        self._thread.start()

    def enqueue(self, channel, monotonic_ns, payload):
        """非阻塞入队；满队列只计 dropped，绝不反压控制 executor。"""
        row = {
            'channel': str(channel),
            'monotonic_ns': int(monotonic_ns),
            'data': payload,
        }
        try:
            self.queue.put_nowait(row)
        except queue.Full:
            with self._lock:
                self.dropped += 1
            return False
        with self._lock:
            self.enqueued += 1
            self.max_queue_depth = max(
                self.max_queue_depth, self.queue.qsize(),
            )
        return True

    def snapshot(self):
        with self._lock:
            return {
                'enqueued': self.enqueued,
                'written': self.written,
                'dropped': self.dropped,
                'max_queue_depth': self.max_queue_depth,
                'queue_depth': self.queue.qsize(),
                'backpressure': self.dropped > 0,
            }

    def close(self, timeout_sec=10.0):
        """等待队列排空，最终 flush+fsync；writer 异常必须传播给调用方。"""
        if self._thread is None:
            return self.snapshot()
        self._stop.set()
        self._thread.join(timeout=float(timeout_sec))
        if self._thread.is_alive():
            raise TimeoutError('recorder writer did not drain before timeout')
        if self._error is not None:
            raise RuntimeError('recorder writer failed') from self._error
        return self.snapshot()

    def _write_row(self, stream, row, *, count=True):
        stream.write(json.dumps(
            row, separators=(',', ':'), allow_nan=False,
        ) + '\n')
        if count:
            with self._lock:
                self.written += 1

    def _run(self):
        try:
            with self.path.open('w', encoding='utf-8') as stream:
                pending = []
                last_flush = time.monotonic()
                while not self._stop.is_set() or not self.queue.empty():
                    try:
                        row = self.queue.get(timeout=self.flush_interval_sec)
                        pending.append(row)
                    except queue.Empty:
                        pass
                    now = time.monotonic()
                    if pending and (
                        len(pending) >= self.batch_size
                        or now - last_flush >= self.flush_interval_sec
                        or (self._stop.is_set() and self.queue.empty())
                    ):
                        if self.writer_delay_sec > 0.0:
                            time.sleep(self.writer_delay_sec)
                        for item in pending:
                            self._write_row(stream, item)
                            self.queue.task_done()
                        pending.clear()
                        stream.flush()
                        last_flush = time.monotonic()

                stats = self.snapshot()
                if stats['backpressure']:
                    self._write_row(stream, {
                        'channel': 'RECORDER_BACKPRESSURE',
                        'monotonic_ns': time.monotonic_ns(),
                        'data': stats,
                    }, count=False)
                self._write_row(stream, {
                    'channel': 'RECORDER_FINAL',
                    'monotonic_ns': time.monotonic_ns(),
                    'data': stats,
                }, count=False)
                stream.flush()
                os.fsync(stream.fileno())
        except Exception as exc:  # pragma: no cover - 错误由 close 统一传播
            self._error = exc
