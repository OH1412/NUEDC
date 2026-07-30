$ErrorActionPreference = "Stop"
$Port = 5600
$SubnetPrefix = "192.168.50."
$FirewallRuleName = "NUEDC H Video UDP 5600"
$SdpPath = Join-Path $PSScriptRoot "receiver.sdp"

Write-Host "NUEDC H PC video receiver" -ForegroundColor Cyan

$Addresses = @(
    Get-NetIPAddress -AddressFamily IPv4 |
        Where-Object {
            $_.IPAddress.StartsWith($SubnetPrefix) -and
            $_.AddressState -ne "Duplicate"
        }
)

if ($Addresses.Count -eq 0) {
    Write-Host ""
    Write-Host "未找到 192.168.50.x 地址。" -ForegroundColor Red
    Write-Host "请先让PC连接Wi-Fi：NUEDC-H，然后重新运行本脚本。"
    exit 2
}

$PcAddress = $Addresses[0].IPAddress
Write-Host "PC接收地址：$PcAddress"
Write-Host "UDP端口：$Port"
Write-Host ""
Write-Host "Jetson推流参数应为：--stream-host $PcAddress --stream-port $Port" `
    -ForegroundColor Green

$Identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$Principal = New-Object Security.Principal.WindowsPrincipal($Identity)
$IsAdministrator = $Principal.IsInRole(
    [Security.Principal.WindowsBuiltInRole]::Administrator
)

if ($IsAdministrator) {
    $ExistingRule = Get-NetFirewallRule `
        -DisplayName $FirewallRuleName `
        -ErrorAction SilentlyContinue
    if ($null -eq $ExistingRule) {
        New-NetFirewallRule `
            -DisplayName $FirewallRuleName `
            -Direction Inbound `
            -Action Allow `
            -Protocol UDP `
            -LocalPort $Port `
            -Profile Any | Out-Null
        Write-Host "已添加Windows防火墙UDP $Port 入站规则。"
    } else {
        Write-Host "Windows防火墙规则已存在。"
    }
} else {
    Write-Host ""
    Write-Host "当前不是管理员，无法自动添加防火墙规则。" `
        -ForegroundColor Yellow
    Write-Host "若收不到画面，请右键 start_receiver.bat，选择“以管理员身份运行”。"
}

$VlcCandidates = @(
    (Join-Path $env:ProgramFiles "VideoLAN\VLC\vlc.exe"),
    (Join-Path ${env:ProgramFiles(x86)} "VideoLAN\VLC\vlc.exe")
)

$VlcCommand = Get-Command "vlc.exe" -ErrorAction SilentlyContinue
if ($null -ne $VlcCommand) {
    $VlcPath = $VlcCommand.Source
} else {
    $VlcPath = $VlcCandidates |
        Where-Object { Test-Path $_ } |
        Select-Object -First 1
}

if ([string]::IsNullOrWhiteSpace($VlcPath)) {
    Write-Host ""
    Write-Host "未找到VLC。" -ForegroundColor Red
    Write-Host "请先安装VLC，再重新运行 start_receiver.bat。"
    exit 3
}

if (-not (Test-Path $SdpPath)) {
    Write-Host "缺少接收配置：$SdpPath" -ForegroundColor Red
    exit 4
}

Write-Host ""
Write-Host "正在启动VLC；请保持该窗口和VLC运行。" -ForegroundColor Cyan
$QuotedSdpPath = '"' + $SdpPath + '"'
Start-Process `
    -FilePath $VlcPath `
    -ArgumentList @(
        "--network-caching=200",
        "--live-caching=200",
        "--clock-jitter=0",
        "--clock-synchro=0",
        $QuotedSdpPath
    )
