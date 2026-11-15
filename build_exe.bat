@echo off
REM Compila QuadSim como ejecutable de Windows (carpeta autocontenida).
REM Requisitos: Python 3.10+ en el PATH. Uso: doble clic o "build_exe.bat" en una terminal.
setlocal
cd /d "%~dp0"
if not exist .venv (
    python -m venv .venv || goto :error
)
call .venv\Scripts\activate.bat || goto :error
python -m pip install --upgrade pip
pip install -r requirements.txt pyinstaller || goto :error
pyinstaller --noconfirm packaging\QuadSim.spec || goto :error
powershell -NoProfile -Command "Compress-Archive -Force -Path dist\QuadSim -DestinationPath QuadSim-1.1.0-win64.zip" || goto :error
echo.
echo Listo: dist\QuadSim\QuadSim.exe  y  QuadSim-1.1.0-win64.zip
goto :eof
:error
echo La compilacion fallo. Revise los mensajes anteriores.
exit /b 1
