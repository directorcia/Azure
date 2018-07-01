## CIAOPS
## Script provided as is. Use at own risk. No guarantees or warranty provided.

## Prerequisites = 1
## 1. Ensure azurerm module installed or updated

## ensure that install-azurerm msonline has been run
## ensure that update-azurerm msonline has been run to get latest module
## Module = https://www.powershellgallery.com/packages/AzureRM/
## Latest version 6.3.0 June 2018
Import-module azurerm

Login-azurermaccount

## Select ARM account
$subscription=get-azurermsubscription

if ($subscription.count -lt 2){         ## See if there are multiple subscriptions in tenant
    ## Only one subscription in tenant so set this as default
    Set-AzureRmContext -SubscriptionID $subscription    
}
else {
    # More than one subscription, set the first one as current
    Set-AzureRmContext -SubscriptionID $subscription.Id[0]
    ## Select-AzureSubscription  -SubscriptionID $subscription.Id[0] –Default     
}

## Get-AzureSubscription -SubscriptionDataFile "C:\Temp\MySubscriptions.xml

write-host -foregroundcolor green "Now connected to Azure"