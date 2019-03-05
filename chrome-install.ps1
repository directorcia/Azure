## CIAOPS
## Script provided as is. Use at own risk. No guarantees or warranty provided.

## Description
## Script to download and install Chrome

## Source - CIAOPS Patron Repository

## Variables
$systemmessagecolor = "cyan"
$processmessagecolor = "green"
$warningmessagecolor = "yellow"

Clear-Host

write-host -foregroundcolor $systemmessagecolor "Script started`n"

$Path = $env:TEMP
$Installer = "chrome_installer.exe"
Invoke-WebRequest "http://dl.google.com/chrome/install/375.126/chrome_installer.exe" -OutFile $Path\$Installer
Start-Process -FilePath $Path\$Installer -Args "/silent /install" -Verb RunAs -Wait
Remove-Item $Path\$Installer

write-host -foregroundcolor $systemmessagecolor "Script complete`n"