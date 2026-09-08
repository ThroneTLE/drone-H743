"""Queue background file work back to Tk; discard superseded/closed results."""
import queue
import threading


class LogJobs:
    def __init__(self, owner, error):
        self.owner, self.error = owner, error
        self.events = queue.Queue()
        self.versions = {}
        self.closed = False
        self.timer = owner.after(50, self.poll)
        owner.bind("<Destroy>", self._destroy, add="+")

    def submit(self, key, work, done):
        version = self.versions.get(key, 0) + 1
        self.versions[key] = version

        def run():
            try:
                result = (True, work())
            except Exception as exc:
                result = (False, str(exc))
            self.events.put((key, version, done, result))

        threading.Thread(target=run, daemon=True).start()

    def cancel(self, key):
        self.versions[key] = self.versions.get(key, 0) + 1

    def poll(self):
        if self.closed:
            return
        while True:
            try:
                key, version, done, (ok, result) = self.events.get_nowait()
            except queue.Empty:
                break
            if self.versions.get(key) != version:
                continue
            try:
                if ok:
                    done(result)
                else:
                    self.error(result)
            except Exception as exc:
                self.error(str(exc))
        self.timer = self.owner.after(50, self.poll)

    def _destroy(self, event):
        if event.widget is self.owner:
            self.closed = True
            self.owner.after_cancel(self.timer)
