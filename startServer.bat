@echo off

setlocal

rem Define the URL to check
set url=http://localhost:8080

rem Check if the web server is available
curl -s -o nul %url%
if %errorlevel% neq 0 (
    echo Web server at %url% is not available.
    echo Starting C:\STRstudio\sito-studio-win.exe
    start C:\STRstudio\sito-studio-win.exe
) else (
    echo Web server at %url% is available.
)

endlocal
