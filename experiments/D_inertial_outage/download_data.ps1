# 在仓库根目录运行；仅下载约4MB的惯性数据包，其中包含配套真值。
param([ValidateSet('easy', 'hard')][string]$Difficulty = 'easy')
$ErrorActionPreference = 'Stop'
$dataRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\..\data_D\downloads'))
New-Item -ItemType Directory -Force -Path $dataRoot | Out-Null
$target = Join-Path $dataRoot "ArchVizTinyHouseDay_${Difficulty}_imu.zip"
$hashes = @{easy='cf34be5926c7fa24006322654586d925913682aac879783d789e270b4771058c'; hard='d6b8bc740ea1a97204f85f890562dbb2717de3c0ed0b49faaf16e8af6dcee801'}
$expected = $hashes[$Difficulty]
if (-not (Test-Path -LiteralPath $target)) {
    Write-Host '正在下载官方IMU和真值数据……'
    Invoke-WebRequest -Uri "https://huggingface.co/datasets/theairlabcmu/tartanair2/resolve/main/ArchVizTinyHouseDay/Data_${Difficulty}/imu.zip" -OutFile $target
}
if ((Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash.ToLower() -ne $expected) {
    throw '校验失败，文件可能未下载完整，请检查该文件。'
}
Write-Host "数据校验通过：$target"
