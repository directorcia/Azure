## CIAOPS
## Script provided as is. Use at own risk. No guarantees or warranty provided.

## Prerequisites = 1
## 1. Ensure azurerm module installed or updated

## ensure that install-module msonline has been run
## ensure that update-module msonline has been run to get latest module
Import-module azurerm

Login-azurermaccount

## Select ARM account
$subscription=get-azurermsubscription
## determine if there multiple subscriptions
## if not then select just that one
## if so select the first one

Set-AzureRmContext -SubscriptionID $subscription.Id[0]  ## select first subscription available

write-host -foregroundcolor green "Now connected to Azure"