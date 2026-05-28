import os
import subprocess
import shutil

model_files = ["Tilt_Switch_Digital.obj", "Tilt_Switch_Digital.mtl"]
data_files = ["icon.ico"] + model_files   # включаем иконку

cmd = [
    "pyinstaller",
    "--onefile",
    "--windowed",
    "--name", "TiltMonitor",
]

# Иконка для самого exe-файла (как раньше)
if os.path.exists("icon.ico"):
    cmd.extend(["--icon", "icon.ico"])

# Скрытые импорты
cmd.extend([
    "--hidden-import", "PyQt5.sip",
    "--hidden-import", "OpenGL.platform.win32",
])

# Добавляем все data-файлы (модель и иконку)
for f in data_files:
    if os.path.exists(f):
        cmd.extend(["--add-data", f"{f};."])

cmd.append("digital_tilt.py")

print("Сборка...")
subprocess.run(cmd)

if os.path.exists("dist/TiltMonitor.exe"):
    for f in model_files:
        if os.path.exists(f):
            shutil.copy(f, "dist/")
    print("✅ Готово! Файл в папке dist/")
else:
    print("❌ Ошибка сборки")