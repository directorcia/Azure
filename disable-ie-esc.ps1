## CIAOPS
## Script provided as is. Use at own risk. No guarantees or warranty provided.

## Description
## Script to disable Internet Explorer Enhanced Security mode on a server

## Source - CIAOPS Patron Repository

## Variables
$systemmessagecolor = "cyan"
$processmessagecolor = "green"
$warningmessagecolor = "yellow"

Clear-Host

write-host -foregroundcolor $systemmessagecolor "Script started`n"

$AdminKey = "HKLM:\SOFTWARE\Microsoft\Active Setup\Installed Components\{A509B1A7-37EF-4b3f-8CFC-4F3A74704073}"
$UserKey = "HKLM:\SOFTWARE\Microsoft\Active Setup\Installed Components\{A509B1A8-37EF-4b3f-8CFC-4F3A74704073}"
Set-ItemProperty -Path $AdminKey -Name "IsInstalled" -Value 0
Set-ItemProperty -Path $UserKey -Name "IsInstalled" -Value 0
Stop-Process -Name Explorer
Write-Host "IE Enhanced Security Configuration (ESC) has been disabled." -ForegroundColor $processmessagecolor

write-host -foregroundcolor $systemmessagecolor "Script complete`n"
