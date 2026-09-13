from pathlib import Path
import ast
import shutil
import subprocess
import tempfile
import sys
root=Path.cwd();work=Path(tempfile.mkdtemp(prefix='adc-stop-negative-'))
sys.path.insert(0,str(root))
# Materialize the exact test harness with the prior committed BSP only.
source=(root/'tests/test_battery_runtime.py').read_text(encoding='utf-8')
source=source.replace("str(ROOT/path) for path in ('App/Src/app_current.c','BSP/Src/bsp_current.c',",
                      "str(work/'old_bsp_current.c') if path=='BSP/Src/bsp_current.c' else str(ROOT/path) for path in ('App/Src/app_current.c','BSP/Src/bsp_current.c',")
(work/'old_bsp_current.c').write_bytes(subprocess.check_output(['git','show','fbf54ca9:BSP/Src/bsp_current.c']))
space={'__file__':str(root/'tests/test_battery_runtime.py'),'work':work}
exec(compile(source,str(root/'tests/test_battery_runtime.py'),'exec'),space)
factory=type('Factory',(),{'mktemp':lambda self,name:work})()
try:
    space['firmware_battery'].__wrapped__(factory)
except AssertionError as error:
    print('Prior committed BSP fails the new stop-failure contract:')
    print(error)
else:
    raise AssertionError('Negative control unexpectedly passed')
