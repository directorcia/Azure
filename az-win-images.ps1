<# CIAOPS
Script provided as is. Use at own risk. No guarantees or warranty provided.

Description
Script designed to display available Windows IaaS Images

Prerequisites = 1
1. Ensure azurerm module installed or updated

ensure that install-azurerm msonline has been run
ensure that update-azurerm msonline has been run to get latest module
Module = https://www.powershellgallery.com/packages/AzureRM/
Latest version 6.13.1 21 November 2018

Allow custom scripts to run just for this instance
set-executionpolicy -executionpolicy bypass -scope currentuser -force
#>

## Variables
$systemmessagecolor = "cyan"
$processmessagecolor = "green"
$errormessagecolor = "red"
$warnmessagecolor = "yellow"

$location = "westus2" ## Azure region to check
$pubName = "MicrosoftWindowsServer"
$offerName = "WindowsServer"

Clear-Host

write-host -foregroundcolor $systemmessagecolor "Script started`n"

## Names of all VM image publishers
write-host -foregroundcolor $processmessagecolor "All VM Image Publishers`n"
$images = Get-AzureRMVMImagePublisher -Location $location
$images.publishername

Read-Host -prompt "`nPress Enter"
Clear-Host

## Names of VM Images Published for MicrosoftWindowsServer
write-host -foregroundcolor $processmessagecolor "All VM Image Families Published by Publisher`n"
$imageoffer = Get-AzureRMVMImageOffer -Location $location -Publisher $pubName 
$imageoffer.Offer

Read-Host -prompt "`nPress Enter"
Clear-Host

## Names of VM Images offered by Image Publisher i.e. WindowsServer
write-host -foregroundcolor $processmessagecolor "All VM Images Offered by Image Publisher`n"
$imagesku=Get-AzureRMVMImageSku -Location $location -Publisher $pubName -Offer $offerName 
$imagesku.skus

write-host -foregroundcolor $systemmessagecolor "`nScript finished"