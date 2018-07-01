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
$subscriptionname = $subscription.name[0]   ## Select first listed Azure subscription

Select-azurermsubscription -subscriptionname $subscriptionname
write-host -foregroundcolor green "Now connected to Azure"