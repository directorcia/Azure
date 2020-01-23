<# CIAOPS
Script provided as is. Use at own risk. No guarantees or warranty provided.

Description - Script designed to login to Azure resources
Source - https://github.com/directorcia/Azure/blob/master/az-connect.ps1

Prerequisites = 1
1. Ensure azurerm module installed or updated

ensure that install-az msonline has been run
ensure that update-az msonline has been run to get latest module

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

write-host -foregroundcolor $processmessagecolor "Azure PowerShell module loading...."
Import-module az
write-host -foregroundcolor $processmessagecolor "Azure PowerShell module loaded"

connect-azaccount

## Select ARM account

$subscription = get-azsubscription

if ($subscription.count -lt 2) {
    ## See if there are multiple subscriptions in tenant
    ## Only one subscription in tenant so set this as default
    Set-AzContext -SubscriptionID $subscription    
}
else {
    # More than one subscription, set the first one as current
    write-host -foregroundcolor $processmessagecolor $subscription.count, "subscriptions detected."
    write-host -foregroundcolor $warnmessagecolor "Selecting subscription = ", $subscription.name[0]
    Set-AzContext -SubscriptionID $subscription.Id[0]
    ## Select-AzureSubscription  -SubscriptionID $subscription.Id[0] –Default     
}

## Get-AzureSubscription -SubscriptionDataFile "C:\Temp\MySubscriptions.xml

write-host -foregroundcolor $systemmessagecolor "Script finished"