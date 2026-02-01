$ErrorActionPreference = "Stop"

$BUILD_DIR = "build_output"
if (Test-Path $BUILD_DIR) {
    Remove-Item -Path $BUILD_DIR -Recurse -Force
}
New-Item -ItemType Directory -Path $BUILD_DIR | Out-Null

Write-Host "Compiling with Map File Generation..."
$MAP_FILE = "$PWD/$BUILD_DIR/output.map"
$MAP_FILE = $MAP_FILE -replace "\\", "/"

arduino-cli compile --fqbn arduino:avr:uno `
    --libraries libraries `
    --build-property "compiler.c.elf.extra_flags=-Wl,-Map=""$MAP_FILE""" `
    --output-dir $BUILD_DIR `
    app/kv_db_demo

if ($LASTEXITCODE -ne 0) {
    Write-Error "Compilation failed."
    exit 1
}

Write-Host "`nCompilation Successful."
Write-Host "Map file generated at: $MAP_FILE"

# Find Toolchain Path
$TOOLCHAIN_PATH = ""
$AVR_SIZE = Get-Command avr-size -ErrorAction SilentlyContinue
if (-not $AVR_SIZE) {
    $LOCALAPPDATA = [Environment]::GetFolderPath("LocalApplicationData")
    $POTENTIAL_PATH = Get-ChildItem -Path "$LOCALAPPDATA\Arduino15\packages\arduino\tools\avr-gcc" -Recurse -Filter "avr-size.exe" | Select-Object -First 1
    if ($POTENTIAL_PATH) {
        $AVR_SIZE = $POTENTIAL_PATH.FullName
        $TOOLCHAIN_PATH = $POTENTIAL_PATH.DirectoryName
    }
} else {
    $TOOLCHAIN_PATH = Split-Path $AVR_SIZE.Source
}

# 1. Run avr-size
if ($AVR_SIZE) {
    Write-Host "`n--- AVR Size Analysis ---"
    & $AVR_SIZE -C --mcu=atmega328p "$BUILD_DIR/kv_db_demo.ino.elf"
} else {
    Write-Warning "avr-size not found path. Skipping standard size report."
}

# 2. Run avr-nm for Top Consumers
$AVR_NM = if ($TOOLCHAIN_PATH) { Join-Path $TOOLCHAIN_PATH "avr-nm.exe" } else { "" }
if (Test-Path $AVR_NM) {
    Write-Host "`n--- Top RAM Consumers (Data + BSS) ---"
    # -C: demangle, --size-sort: sort by size, -r: reverse (largest first), -S: print size, -t d: decimal
    # We want symbols in Data (D, d) and BSS (B, b)
    $output = & $AVR_NM -C --size-sort -r -S -t d "$BUILD_DIR/kv_db_demo.ino.elf"
    
    $consumers = @()
    foreach ($line in $output) {
        # Format: Address Size Type Name
        # e.g. 00800100 00000100 B my_large_buffer
        if ($line -match "^[0-9a-fA-F]+\s+(\d+)\s+([dDbB])\s+(.+)$") {
            $consumers += [PSCustomObject]@{
                Size = [int]$matches[1]
                Type = $matches[2]
                Name = $matches[3]
            }
        }
    }

    $consumers | Select-Object -First 20 | Format-Table -AutoSize
} else {
    Write-Warning "avr-nm not found. Skipping detailed RAM analysis."
}
