# SPDX-License-Identifier: MIT
#
# One-time host setup for Renode networking on Windows (run elevated):
#   1. installs the TAP-Windows6 driver and tapctl.exe from the OpenVPN MSI
#      (features: OpenVPN core + Drivers.TAPWindows6 only; no service, no GUI)
#   2. creates the TAP adapter "rtoos-tap"
#   3. assigns the host side of the emulation network: 192.168.77.1/24
#   4. allows inbound traffic on that adapter only (firewall rule "rtoos emulation network")
#
# Undo: Remove-NetFirewallRule -DisplayName "rtoos emulation network";
#       C:\opt\openvpn\bin\tapctl.exe delete rtoos-tap; uninstall "OpenVPN" in Windows settings
#
# The emulated Factory Control Unit uses 192.168.77.2 (see emulation/renode/fcu-net.resc).
#
#   powershell -ExecutionPolicy Bypass -File emulation\renode\setup-tap.ps1 -Msi <OpenVPN-...-amd64.msi>
param(
    [Parameter(Mandatory = $true)][string]$Msi,
    [string]$InstallDir = "C:\opt\openvpn\",
    [string]$Adapter = "rtoos-tap",
    [string]$HostAddress = "192.168.77.1",
    [string]$Mask = "255.255.255.0"
)
$ErrorActionPreference = "Stop"
$log = Join-Path $env:TEMP "rtoos-setup-tap.log"
Start-Transcript -Path $log -Force | Out-Null
try {
    $tapctl = Join-Path $InstallDir "bin\tapctl.exe"
    if (-not (Test-Path $tapctl)) {
        Write-Host "Installing TAP-Windows6 driver from $Msi ..."
        $msiLog = Join-Path $env:TEMP "rtoos-openvpn-msi.log"
        $p = Start-Process msiexec.exe -Wait -PassThru -ArgumentList @(
            "/i", "`"$Msi`"", "/qn", "/norestart",
            "ADDLOCAL=OpenVPN,Drivers,Drivers.TAPWindows6",
            "PRODUCTDIR=`"$InstallDir`"",
            "/l*v", "`"$msiLog`"")
        if ($p.ExitCode -ne 0 -and $p.ExitCode -ne 3010) { throw "msiexec failed: $($p.ExitCode) (log: $msiLog)" }
    }

    $existing = Get-NetAdapter -Name $Adapter -ErrorAction SilentlyContinue
    if (-not $existing) {
        Write-Host "Creating TAP adapter $Adapter ..."
        & $tapctl create --hwid root\tap0901 --name $Adapter
        if ($LASTEXITCODE -ne 0) { throw "tapctl create failed: $LASTEXITCODE" }
    }

    Write-Host "Assigning $HostAddress/$Mask to $Adapter ..."
    & netsh interface ipv4 set address name="$Adapter" static $HostAddress $Mask
    if ($LASTEXITCODE -ne 0) { throw "netsh failed: $LASTEXITCODE" }

    # The TAP adapter is an "unidentified network" (Public profile): inbound traffic
    # from the emulated device (SNTP requests to the test server, mDNS answers) would
    # be dropped. Allow inbound on this adapter only.
    $rule = "rtoos emulation network"
    if (-not (Get-NetFirewallRule -DisplayName $rule -ErrorAction SilentlyContinue)) {
        Write-Host "Adding firewall rule '$rule' (inbound, interface $Adapter only) ..."
        New-NetFirewallRule -DisplayName $rule -Direction Inbound -Action Allow -InterfaceAlias $Adapter `
            -Profile Any -Description "Inbound traffic from the Renode-emulated FCU on the host-only TAP network" | Out-Null
    }
    Get-NetAdapter -Name $Adapter | Format-List Name, InterfaceDescription, Status, MacAddress
    Write-Host "OK"
}
finally {
    Stop-Transcript | Out-Null
}
