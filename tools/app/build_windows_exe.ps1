$repoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
Push-Location $repoRoot

try {
    uv sync
    uv run pyinstaller --noconfirm --clean --name LGRSimWorkbench --onedir --add-data "src/app/static;app/static" src/app/run_local_app.py

    Write-Host "Build complete. Folder output: dist/LGRSimWorkbench"
    Write-Host "Zip dist/LGRSimWorkbench and share it."
}
finally {
    Pop-Location
}
