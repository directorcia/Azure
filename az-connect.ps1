<# CIAOPS
Script provided as is. Use at own risk. No guarantees or warranty provided.

Description
Script designed to login to Azure resources

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

Clear-Host

write-host -foregroundcolor $systemmessagecolor "Script started`n"

Import-module azurerm
write-host -foregroundcolor $processmessagecolor "Azure PowerShell module loaded"

Login-azurermaccount

## Select ARM account

$subscription=get-azurermsubscription

if ($subscription.count -lt 2){         ## See if there are multiple subscriptions in tenant
    ## Only one subscription in tenant so set this as default
    Set-AzureRmContext -SubscriptionID $subscription    
}
else {
    # More than one subscription, set the first one as current
    write-host -foregroundcolor $processmessagecolor $subscription.count,"subscriptions detected."
    write-host -foregroundcolor $warnmessagecolor "Selecting subscription = ", $subscription.name[0]
    Set-AzureRmContext -SubscriptionID $subscription.Id[0]
    ## Select-AzureSubscription  -SubscriptionID $subscription.Id[0] –Default     
}

## Get-AzureSubscription -SubscriptionDataFile "C:\Temp\MySubscriptions.xml

write-host -foregroundcolor $systemmessagecolor "Script finished"